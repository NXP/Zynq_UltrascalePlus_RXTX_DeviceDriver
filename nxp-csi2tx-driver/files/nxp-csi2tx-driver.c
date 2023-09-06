/*/NXP Radar Driver
 * SPDX-License-Identifier:GPL-2.0
 * Copyright (C) 2019 NXP
 *
 * AUTHOR: NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "nxp-csi2tx-driver.h"
#ifdef MODULE_PARAM_EN
#include <linux/moduleparam.h>
#endif

#define NUM_UUTS  1								//Max number of supported UUTs
#define NUM_ADC_CHANNELS 4						//Number of ADC channels per UUT
#define NUM_DEVICES (1 + NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS))	//Each uut will have a adc device and a frame device (do we need a status device? for interrupts and so on ...)
#define SUCCESS 0								//Success code
#define TRUE 1
#define FALSE 0
#define MY_ADC_FRAME(I) ((I - NUM_UUTS -1)/NUM_ADC_CHANNELS)		//Macro to find out the corresponding frame device from adc device minor 
#define METADATA_SIZE   64
#define MAX_FRAMES_LOAD  4 /*Max Number of frames to be loaded in to DDR memory is 4*/

#ifdef MODULE_PARAM_EN
static int USBEn = 0 , radar_cnt = 0;
module_param(USBEn, int, S_IRUSR|S_IWUSR);                      //integer value
module_param(radar_cnt, int, S_IRUSR|S_IWUSR);                      //integer value
#endif

//***************** Device structures ********************
struct csi2tx_fpga_dev {
    struct mutex csi2tx_mutex; 
    struct cdev cdev;
};
struct csi2tx_frame_dev {
    uint64_t *frame_data;
    uint32_t frame_address;
    uint32_t frame_size;
};
struct csi2tx_adc_dev{
    struct csi2tx_frame_dev *my_frame;
    struct mutex csi2tx_mutex; 
    struct cdev cdev;
    uint8_t adc_code;
};
//================= 
struct nxp_datacap_usb_rd_wr_local {
    struct work_struct work;        /* used in workqueue */
    uint32_t frame_address;
    uint32_t frame_size;
    struct device   *dev_ptr;
};

//================== Work queue ==============================
/* Work structure */
static struct work_struct csi2tx_workq;
static atomic_t counter_th;
static struct nxp_datacap_usb_rd_wr_local *pcsi2tx_usb_dev = NULL;
static void workqueue_fn(struct work_struct *work); 
 
static char filename[1024] = { 0 };
static struct file *gp_usbfp=NULL;
static loff_t gfp_pos = 0;
static uint32_t radar_cycle_cnt = 0 , numFramesTransmit = 0;
static int resetDone = 0; // Stores application FPGA resetDone in user space
static int usbCapEn  = 0; // To Capture to USB SSD the files
static int usbReadComplete  = 0; // To Inform the Status of USB Write Complete 
static int radarCycConst = 0;
static uint8_t FrameStatus = NO_FRAME_INT_TRIGGERED, readStatus = 0; /* IRQ Status , buffer read status*/ 
static uint32_t frameNumber = 0, FrameSize = 0;
static uint32_t metadata_cnt = 0;
static uint32_t Nframe_load_cnt = MAX_FRAMES_LOAD;
static struct metadata_header metaBuff;
static struct streamAdcInfo streamAdc;
static uint32_t msdelay = 0;
//================== Device Variables ========================
static dev_t csi2tx_dev_major , device_number;									//Device identifier
static struct csi2tx_frame_dev *pcsi2tx_frame_devices = NULL;		//Collection of Frame devices used by ADC devices
static struct csi2tx_adc_dev *pcsi2tx_adc_devices = NULL;			//Collection of ADC devices
static struct csi2tx_fpga_dev csi2tx_control_dev;					//Single instance of custom PL controller
static struct class *pcsi2tx_class = NULL;						//Driver class
//******************* Interrupt codes *********************
static int FRAME_INTERRUPT_TX = 0;					// Place to store OS interrupt code for SPI HW int	
//=================== Kernel operation variables ===================
static unsigned int *pframePtr = NULL;
static unsigned int frame_ptr;
//static unsigned int detectedChirps;
static struct task_struct *ptask = NULL;
static struct kernel_siginfo info;

#define LOOP_BACK_TEST
#define CSI2_TX_SUBSYSTEM_START_ADDR    0xA0002000
#define CSI2_TX_SUBSYSTEM_END_ADDR      0xA0004000
#ifdef LOOP_BACK_TEST
static unsigned int *lbPtr = NULL; // Loop back 
#endif
#define TASK 

/*****************************************************************************/
#ifdef TIME_STAMP_EN
#include <linux/timekeeping.h>
#include <linux/jiffies.h> 
static uint64_t before_schedule_timestamp = 0, after_schedule_timestamp = 0;
static unsigned long t1, t2, diff, msec;
#endif /*TIME_STAMP_EN*/
/*******************************************************************************/

static int USBSSD_DataTransmit(struct streamAdcInfo streamAdc);

/*
*        sigVal = 11;  0xB  File Open ERROR. (Work queue function).
*        sigVal = 12;  0xC  Memory Map failed.(Work queue function).
*        sigVal = 13;  0xD  I/O read error (File read error).(Work queue function).
*        sigVal = 14;  0xE	Error in Read from File (Read Failed) / No Data left on USB file.(Work queue function).
*        sigVal = 15;  0xF	Read success. (Work queue function). 
**/
static void send_signal_userSpace(int sigVal)
{
    int ret = 0;

    if (ptask == NULL)
    {
        pr_info("[%s:%d]  TX Task is NULL (sigVal=%d)\n", __FILE__, __LINE__,sigVal);
    }
    else
    {
        info.si_int = sigVal;   // real time signals may have 32 bits of data.
        ret = send_sig_info(SIGUSR2, &info, ptask); // send the signal
        if (ret < 0)
        {
            pr_info("[%s:%d] Error sending signal sigval=%d\n", __FILE__, __LINE__,sigVal);
        }
    }

    return;
}

//====================== Work Queue function ======================================
/*Workqueue Function*/
static void workqueue_fn(struct work_struct *work)
{
    int ret = 0,sigVal = 0;
    ssize_t nbytes = 0;
    uint8_t const *pbuffer = NULL;
    uint32_t frameLength = 0;
    uint64_t *frame_data = NULL;
    uint8_t current_frameIdx = 0;
    int err = 0;
    
    #ifdef DEBUG
    pr_info("[%s:%d] Executing Workqueue Function...\n",__FILE__, __LINE__);
    #endif /*DEBUG*/
    #ifdef TIME_STAMP_EN
        t2 = jiffies; // wrq
        after_schedule_timestamp = ktime_get_real_ns();
        diff = t2-t1;
        msec = diff *1000 / HZ;
        printk("\n\n< Work queue scheduling Time in nanoseconds: %llu> < Jiffies mSec =%llu>\n",(after_schedule_timestamp - before_schedule_timestamp));
    #endif /* TIME_STAMP_EN */

    gp_usbfp = filp_open(filename, O_RDONLY | O_APPEND | O_LARGEFILE, S_IRUSR);
    if (IS_ERR(gp_usbfp))
    {
        pr_err("[%s:%d] File open error.....\n",__FILE__,__LINE__);
        sigVal = 11; /* File Open ERROR */
        send_signal_userSpace(sigVal);
        usbReadComplete = FALSE;
        radar_cycle_cnt = 0;
        FrameStatus = IN_OUT_ERROR;
        return;
    }
    else
    {
        msleep(msdelay);
        current_frameIdx = ioread32(pframePtr + (CSI2TX_FRAME_PTR / 4));
        if(Nframe_load_cnt < radarCycConst)
        {
            current_frameIdx = (current_frameIdx -1) & 3; 
            pcsi2tx_usb_dev->frame_address = ioread32(pframePtr + (CSI2TX_MEM_FRAME + (current_frameIdx * 4)) / 4);
            pcsi2tx_usb_dev->frame_size = ioread32(pframePtr + (CSI2TX_FRAME_LEN / 4));
           #ifdef DEBUG
            pr_info("[%s:%d]:WQ Frame size=%d current_frameIdx=%d frame_address=0x%X\n", __FILE__, __LINE__,pcsi2tx_usb_dev->frame_size,current_frameIdx, pcsi2tx_usb_dev->frame_address);
            #endif /*DEBUG*/
           frame_data = (uint64_t *)memremap(pcsi2tx_usb_dev->frame_address, MAX_FRAME_BUFF_SIZE, MEMREMAP_WB);
            if (frame_data == NULL)
            {
                /*buffer read status*/ 
                FrameStatus = TX_DMA_ERROR; 
                FrameSize = 0;
                radar_cycle_cnt = 0;
                usbCapEn = 0;
                filp_close(gp_usbfp, NULL);
                gp_usbfp = NULL;
                pr_warn("[%s:%d] frame_dev->frame_data buffer is null \n", __FILE__, __LINE__);
                sigVal = 12; /* 0xC  Memory Map failed.*/
                send_signal_userSpace(sigVal);
                return;
            }

            frameLength =  pcsi2tx_usb_dev->frame_size;
            pbuffer = ((const void *)((uint8_t *)(frame_data)));
            while (frameLength > 0)
            {
                #ifdef DEBUG
                pr_info("[%s:%d]: gfp_pos =%ld\n",__FILE__,__LINE__,gfp_pos);
                #endif /*DEBUG*/

                nbytes = kernel_read(gp_usbfp, pbuffer, frameLength, &gfp_pos);
                if (nbytes == 0)
                {
                    pr_err("[%s:%d] I/O read Error <the data read from USB file.>\n", __FILE__, __LINE__);
                    sigVal = 13; /* 0xD  I/O read error (File read error).*/
                    send_signal_userSpace(sigVal);
                    memunmap(frame_data);
                    filp_close(gp_usbfp, NULL);
                    gp_usbfp = NULL;
                    FrameStatus = IN_OUT_ERROR;
                    FrameSize = 0;
                    radar_cycle_cnt = 0;
                    return;
                }
                else if (nbytes < 0)
                {
                    pr_err("[%s:%d] Error in Read /  No data on USB device file... \n", __FILE__, __LINE__);
                    sigVal = 14; /* 0xE	Error in Read from File (Read Failed) / No Data left on USB file */
                    send_signal_userSpace(sigVal);
                    usbReadComplete = FALSE;
                    memunmap(frame_data);
                    filp_close(gp_usbfp, NULL);
                    gp_usbfp = NULL;
                    FrameStatus = IN_OUT_ERROR;
                    FrameSize = 0;
                    radar_cycle_cnt = 0;
                    return ;
                }
                frameLength -= nbytes;
                pbuffer += nbytes;
                #ifdef DEBUG
                pr_info("[%s:%d]:After Kernel Read gfp_pos =%ld\n", __FILE__, __LINE__, gfp_pos);
                #endif /*DEBUG*/
            }
            #ifdef DEBUG
            pr_info("[%s:%d]: Number of Frames Loaded into DDR =%X \n",__FILE__,__LINE__,Nframe_load_cnt);
            #endif /*DEBUG*/
            Nframe_load_cnt++; /* Next frame */
        }
        if(radar_cycle_cnt != 0)
        {
            FrameSize = pcsi2tx_usb_dev->frame_size;
            #ifdef DEBUG
            pr_info("[%s:%d]:start Frame Transmit Nframes=%d radar_cnt=%d \n",__FILE__,__LINE__,numFramesTransmit,radar_cycle_cnt);
            #endif /*DEBUG*/
            iowrite32(0xFF, pframePtr + (CSI2TX_IRQ_CLR / 4));
            iowrite32(0x1, pframePtr + (CSI2TX_FRAME_START / 4));
        }

        sigVal = 15; /* 0xf	 Read success */
        send_signal_userSpace(sigVal);

        usbReadComplete = TRUE; // Done Success
        memunmap(frame_data);
        err = filp_close(gp_usbfp,NULL);
        if (err)
            pr_err("Error %pe closing the file  \n", ERR_PTR(err));
        else
            gp_usbfp = NULL;
    }
    if((Nframe_load_cnt == radarCycConst) && (numFramesTransmit ==  radarCycConst))
    {
#ifdef DEBUG
        pr_info("[%s:%d]: Completed %d Frames Transmission...\n",__FILE__,__LINE__,numFramesTransmit);
#endif /* DEBUG */
        radar_cycle_cnt = 0;
        usbCapEn = 0;
        gfp_pos = 0; /* resetting the file position to 0 */
        FrameStatus = NO_FRAME_INT_TRIGGERED;
    }
    return;
}

//====================== Custom PL Control Device Functions ========================
static int device_open(struct inode *inode, struct file *filp){
    unsigned int mj = imajor(inode);
    unsigned int mn = iminor(inode);

    struct csi2tx_fpga_dev *dev = &csi2tx_control_dev;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    if (mj != csi2tx_dev_major || mn != 0)
    {
        pr_warn("[%s:%d] [target] " "No device found with minor=%d and major=%d\n",__FILE__,__LINE__, mj, mn);
        return -ENODEV; /* No such device */
    }

    /* store a pointer to struct cfake_dev here for other methods */
    filp->private_data = &csi2tx_control_dev; 

    if (inode->i_cdev != &dev->cdev)
    {
        pr_warn("[%s:%d] [target] open: internal error\n",__FILE__,__LINE__);
        return -ENODEV; /* No such device */
    }

    try_module_get(THIS_MODULE);
    return SUCCESS;
}

static int device_release(struct inode *inode, struct file *file){
#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    module_put(THIS_MODULE);
    return SUCCESS;
}

static ssize_t device_read(struct file *filp, char __user * buffer, size_t length, loff_t * offset){
    int bytes_read;
#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    return bytes_read;
}

static ssize_t device_write(struct file *filp, const char __user * buffer, size_t length, loff_t * offset){
    struct csi2tx_fpga_dev *dev;
#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    dev = (struct csi2tx_fpga_dev *)filp->private_data;

    return length;
}


//============================== FRAME Device Functions ==============================
static int adc_open(struct inode *inode, struct file *filp){
    unsigned int mj = imajor(inode);
    unsigned int mn = iminor(inode);

    struct csi2tx_adc_dev *dev = NULL;
#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */

    if (mj != csi2tx_dev_major || mn <= NUM_UUTS || mn > (NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS)))
    {
        pr_warn("[%s:%d] [target] " "No device found with minor=%d and major=%d\n",__FILE__,__LINE__, mj, mn);
        return -ENODEV; /* No such device */
    }

    /* store a pointer to struct cfake_dev here for other methods */
    dev = &pcsi2tx_adc_devices[mn - (NUM_UUTS + 1)];
    filp->private_data = dev; 

    if (inode->i_cdev != &dev->cdev)
    {
        pr_warn("[%s:%d] [target] open: internal error\n",__FILE__,__LINE__);
        return -ENODEV; /* No such device */
    }

    return 0;
}

static int adc_release(struct inode *inode, struct file *filp){
     iowrite32(0xFF , pframePtr + (CSI2TX_IRQ_CLR/4));
     numFramesTransmit = 0;
#ifdef DEBUG
     pr_info("[%s:%d] frame complete interrupt is cleared \n",__FILE__,__LINE__);
#endif /* DEBUG */
    return 0;
}

static ssize_t adc_write(struct file *filp, const char __user * buffer, size_t length, loff_t * offset){
    uint8_t frame_ptr = 0;
    struct csi2tx_adc_dev *dev;
    struct csi2tx_frame_dev *pframe_dev;
    ssize_t retval;
    dev = (struct csi2tx_adc_dev *)filp->private_data;
    pframe_dev = (struct csi2tx_frame_dev *)dev->my_frame;
    retval = 0;
    
#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    if(usbCapEn == 1){
        usbCapEn = 0;
    }
    
    frame_ptr = ioread32(pframePtr + (CSI2TX_FRAME_PTR/4)); 
    pframe_dev->frame_address = ioread32(pframePtr + (CSI2TX_MEM_FRAME +frame_ptr * 4)/4);
    pframe_dev->frame_size = length;
    
    pframe_dev->frame_data  = (uint64_t *)ioremap(pframe_dev->frame_address, MAX_FRAME_BUFF_SIZE  ); 
    if(pframe_dev->frame_data == NULL)
    {
        pr_info("[%s:%d] pframe_dev->frame_data buffer is null \n",__FILE__,__LINE__);
        return -1;
    }

	copy_from_user(pframe_dev->frame_data, buffer, length);
   
    iounmap(pframe_dev->frame_data);

	return length;
}

//************************************* HW IO ISRs ****************************************
static irqreturn_t data_isr(int irq,void*dev_id) {      
    uint32_t irqRet = 0;
    int ret;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */

    irqRet = ioread32(pframePtr + (CSI2TX_IRQ_STAT/4));
    if ((irqRet & 0xE) != 0x2)
    {
        pr_alert("[%s:%d] Error During Frame transmitted. IRQ: %X!!!\n", __FILE__, __LINE__, irqRet);
        FrameStatus = TX_DMA_ERROR;
    }
    else
    {
        FrameStatus = FRAME_INT_TRIGGERED;
    }

    if(ptask == NULL)
    {
        pr_info("[%s:%d]  TX Task is NULL \n",__FILE__,__LINE__);
    }
    else
    {
        info.si_int = 1;          //real time signals may have 32 bits of data.

        ret = send_sig_info(SIGUSR2, &info, ptask);    //send the signal
        if (ret < 0) {
            pr_info("[%s:%d] Error sending signal =%d\n",__FILE__,__LINE__,ret);
            return IRQ_HANDLED;
        }
    }

    if(usbCapEn)
    {
        if (radar_cycle_cnt != 0)
        {
            radar_cycle_cnt--;
            numFramesTransmit++;
            frameNumber = numFramesTransmit;
            #ifdef DEBUG
            pr_info("started scheduling work queue with radar cycle count=%d  ....\n",radar_cycle_cnt);
            #endif /*DEBUG*/
            #ifdef TIME_STAMP_EN
            t1 =  jiffies; //ISR
            before_schedule_timestamp = ktime_get_real_ns();
            #endif /* TIME_STAMP_EN */
            /*Allocating work to queue*/
            schedule_work(&csi2tx_workq);
        }
    }

out:
    
    return IRQ_HANDLED;
}
//***************** USB SSD Local Functions **************************************
bool dataLoad(struct metadata_header metaBuff,struct file *gp_usbfp)
{
    bool bRet = false;
    uint8_t i = 0, current_frameIdx = 0;
    uint64_t *frame_data = NULL;
    loff_t nbytes;
    uint8_t const *pbuffer = NULL;
    uint32_t frameLength = 0;
    uint32_t sigVal = 0xF;
    
    if (IS_ERR(gp_usbfp))
    {
        pr_err("[%s:%d] File open error.....\n",__FILE__,__LINE__);
        sigVal = 11; /* 0xB File Open error*/
        send_signal_userSpace(sigVal);
        return bRet;
    }
    else
    {
        Nframe_load_cnt = MAX_FRAMES_LOAD > metaBuff.radarCycleCount ?  metaBuff.radarCycleCount : MAX_FRAMES_LOAD;
        #ifdef DEBUG
        pr_info("[%s:%d]: Toatal %d Frames to be loaded into DDR Memory...\n", __FILE__, __LINE__,Nframe_load_cnt);
        #endif /*DEBUG*/

        current_frameIdx = ioread32(pframePtr + (CSI2TX_FRAME_PTR / 4));
        for (i = 0; i < Nframe_load_cnt; i++, current_frameIdx++)
        {
            current_frameIdx &= (MAX_FRAMES_LOAD -1); 
            pcsi2tx_usb_dev->frame_address = ioread32(pframePtr + (CSI2TX_MEM_FRAME + (current_frameIdx * 4)) / 4);
            pcsi2tx_usb_dev->frame_size = ioread32(pframePtr + (CSI2TX_FRAME_LEN / 4));
            
            #ifdef DEBUG
            pr_info("[%s:%d]: Frame size=%d current_frameIdx=%d pcsi2tx_usb_dev->frame_address=0x%X\n", __FILE__, __LINE__,pcsi2tx_usb_dev->frame_size,current_frameIdx, pcsi2tx_usb_dev->frame_address);
            #endif /*DEBUG*/

            frame_data = (uint64_t *)memremap(pcsi2tx_usb_dev->frame_address, MAX_FRAME_BUFF_SIZE, MEMREMAP_WB);
            if (frame_data == NULL)
            {
                pr_info("[%s:%d] frame_data buffer is null \n", __FILE__, __LINE__);
                sigVal = 12; /* 0xC  Memory Map failed.*/
                send_signal_userSpace(sigVal);
                /*buffer read status*/ 
                FrameStatus = IN_OUT_ERROR; 
                FrameSize = 0;
                return bRet;
            }

            frameLength =  pcsi2tx_usb_dev->frame_size;
            
            pbuffer = ((const void *)((uint8_t *)(frame_data)));
            while (frameLength > 0)
            {
                #ifdef DEBUG
                pr_info("[%s:%d]:kernel Read: gfp_pos =%ld\n", __FILE__, __LINE__, gfp_pos);
                #endif /*DEBUG*/
                nbytes = kernel_read(gp_usbfp, pbuffer, frameLength, &gfp_pos);
                if (nbytes == 0)
                {
                    pr_err("[%s:%d] I/O read Error <the data from USB file.>\n", __FILE__, __LINE__);
                    sigVal = 13; /* 0xD  I/O Read Error.*/
                    send_signal_userSpace(sigVal);
                    memunmap(frame_data);
                    filp_close(gp_usbfp, NULL);
                    gp_usbfp = NULL;
                    FrameSize = 0;
                    FrameStatus = IN_OUT_ERROR;
                    return bRet;
                }
                else if (nbytes < 0)
                {
                    pr_err("[%s:%d] Error in Read /  No data on USB device file... \n", __FILE__, __LINE__);
                    sigVal = 14; /* 0xE  File Read Error(No Data left in the file).*/
                    send_signal_userSpace(sigVal);
                    usbReadComplete = FALSE;
                    memunmap(frame_data);
                    filp_close(gp_usbfp, NULL);
                    gp_usbfp = NULL;
                    FrameSize = 0;
                    FrameStatus = IN_OUT_ERROR;
                    return bRet;
                }
                frameLength -= nbytes;
                pbuffer += nbytes;
                
                #ifdef DEBUG
                pr_info("[%s:%d]:After Kernel Read gfp_pos =%ld\n", __FILE__, __LINE__, gfp_pos);
                #endif /*DEBUG*/
            }
        }
        bRet = true;
    }

    /* ONCE Load is success then initialize radar cycle count */
    radar_cycle_cnt =  metaBuff.radarCycleCount;
    memunmap(frame_data);
    #ifdef DEBUG
    pr_info("[%s:%d]: %d Frames Data is Loaded successfully < Radar cycles cnt=%d > ...\n",__FILE__, __LINE__,Nframe_load_cnt,radar_cycle_cnt);
    #endif /*DEBUG*/
    return bRet;
}
void startFrameDataTransmit(struct metadata_header metaBuff,int usbEnFlg)
{
    iowrite32(0xFF, pframePtr + (CSI2TX_IRQ_CLR / 4));
    iowrite32(0x1, pframePtr + (CSI2TX_FRAME_START / 4));
    usbCapEn = usbEnFlg;

    #ifdef DEBUG
    pr_info("[%s:%d]:radar cycles cnt=%d ... 1st Frame is started transimiting....\n",__FILE__, __LINE__,radar_cycle_cnt);
    #endif /*DEBUG*/

    return;
}
//************************************ IOCTL *****************************************
int USBSSD_DataTransmit(struct streamAdcInfo streamAdc)
{
    uint32_t sigVal = 0xF;
    
    msdelay = streamAdc.msdelay;
    memset(filename,0,MAX_FILE_LEN);
    memcpy(filename,streamAdc.filename,MAX_FILE_LEN);
    
    gp_usbfp = filp_open(filename, O_RDONLY | O_APPEND | O_LARGEFILE, S_IRUSR);
    if (IS_ERR(gp_usbfp))
    {
        pr_err("[%s:%d] File open error.....\n",__FILE__,__LINE__);
        sigVal = 11; /* 0xB  File Open error.*/
        send_signal_userSpace(sigVal);
        usbReadComplete = FALSE;
        radar_cycle_cnt = 0;
        return -EFAULT;
    }
    else
    {
        gfp_pos = 0;
        #ifdef DEBUG
        pr_info("[%s:%d]: Initializing file gfp_pos =%ld\n", __FILE__, __LINE__, gfp_pos);
        #endif /*DEBUG*/
        metadata_cnt = kernel_read(gp_usbfp, &metaBuff, sizeof(struct metadata_header), &gfp_pos);
        if (metadata_cnt == 0)
        {
            pr_err("[%s:%d] Meta data I/O read Error <the data from USB file.>\n", __FILE__, __LINE__);
            sigVal = 13; /* 0xD  I/O Read error (meta data Read error).*/
            send_signal_userSpace(sigVal);
            filp_close(gp_usbfp, NULL);
            gp_usbfp = NULL;
            readStatus = 0x3;
            radar_cycle_cnt = 0;
            return -EFAULT;
        }
        else if (metadata_cnt < 0)
        {
            pr_err("[%s:%d] Meta data Read Error /  No data on USB device file... \n", __FILE__, __LINE__);
            sigVal = 14; /* 0xE  Read error (meta data Read error).*/
            send_signal_userSpace(sigVal);
            filp_close(gp_usbfp, NULL);
            gp_usbfp = NULL;
            radar_cycle_cnt = 0;
            return -EFAULT;
        }
        else
        {
            #ifdef DEBUG
            pr_info("[%s:%d]: file gfp_pos =%ld\n", __FILE__, __LINE__, gfp_pos);
            #endif /*DEBUG*/
            radarCycConst = metaBuff.radarCycleCount;
            numFramesTransmit = 0; 
            if (dataLoad(metaBuff, gp_usbfp))
            {
                startFrameDataTransmit(metaBuff, usbCapEn);
            }
            else
            {
                pr_info("[%s:%d]: Data Load failed...\n", __FILE__, __LINE__);
                return -EINVAL;
            }
            filp_close(gp_usbfp, NULL);
            gp_usbfp = NULL;
        }
    }
#ifdef DEBUG
    pr_info("[%s:%d] usbCapEn(%x) is stored successfully\r\n",__FILE__,__LINE__,usbCapEn);
#endif /* DEBUG */
    return PAGE_SIZE;
}

void csi2tx_reg_write( struct csi2_tx_control_reg csi2_tx)
{
    if(csi2_tx.status)
    {
        iowrite32( csi2_tx.vc0_lines, lbPtr + (0x40/4));
    }
    iowrite32( csi2_tx.nframesTrans, pframePtr + (CSI2TX_NFRAMES_TRNSMIT/4));
}

void ioctl_csi2tx_irq_reset( void )
{
        iowrite32( 0xFF, pframePtr + (CSI2TX_IRQ_CLR/4));
}

static long nxp_csi2tx_control_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct csi2_tx_control_reg csi2_tx;

 switch(cmd) {
        case  IOCTL_REG_WRITE_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            csi2tx_reg_write(csi2_tx );
            copy_to_user((struct csi2_tx_control_reg*)arg, &csi2_tx,sizeof(struct csi2_tx_control_reg));
            break;
        case IOCTL_FRAME_START_TX:
            iowrite32( 0x1, pframePtr + (CSI2TX_FRAME_START/4));
            break;
        case IOCTL_FRAME_LEN_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            iowrite32( csi2_tx.frameLen, pframePtr + (CSI2TX_FRAME_LEN/4));
            iowrite32( csi2_tx.line_Len, pframePtr + (CSI2TX_LINE_LEN/4));
            iowrite32( csi2_tx.vert_Lines, pframePtr + (CSI2TX_VERT_LINE/4));
            break;
        case IOCTL_IRQ_READ_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            csi2_tx.irqStat = ioread32(pframePtr +(CSI2TX_IRQ_STAT/4));
            csi2_tx.framesCnt = ioread32(pframePtr +(CSI2TX_FRAMES_TRNSMIT_CNT/4));
            copy_to_user((struct csi2_tx_control_reg*)arg, &csi2_tx, sizeof( struct csi2_tx_control_reg ));
            break;
        case IOCTL_IRQ_RESET_TX :
            ioctl_csi2tx_irq_reset();
            break;
        case IOCTL_BUILD_INFO_READ_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            csi2_tx.buildID = ioread32(pframePtr +(CSI2TX_BUILD_ID/4));
            csi2_tx.buildDate = ioread32(pframePtr +(CSI2TX_BUILD_DATE/4));
            csi2_tx.buildTime = ioread32(pframePtr +(CSI2TX_BUILD_TIME/4));
            csi2_tx.status = true;
            copy_to_user((struct csi2_tx_control_reg*)arg, &csi2_tx, sizeof( struct csi2_tx_control_reg ));
            break;
        case IOCTL_DBG1_DBG2_READ_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            csi2_tx.calSum_dbg1 = ioread32(pframePtr +(CSI2TX_CAL_SUM_DBG1/4));
            csi2_tx.calSum_dbg2 = ioread32(pframePtr +(CSI2TX_CAL_SUM_DBG2/4));
            copy_to_user((struct csi2_tx_control_reg*)arg, &csi2_tx, sizeof( struct csi2_tx_control_reg ));
            break;
        case IOCTL_CONF_SETTINGS_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            iowrite32( csi2_tx.setting, pframePtr + (CSI2TX_SETTINGS/4));
            break;
        case IOCTL_METADATA_HEADER_TX:
            copy_from_user(&streamAdc, (struct streamAdcInfo *)arg, sizeof(struct streamAdcInfo));
            USBSSD_DataTransmit(streamAdc);
            break;
        default:
            pr_info("[%s:%d] Unknown IOCTL ...\n",__FILE__,__LINE__);
            break;
    }

    return 0;
}

//============================ File Operation Definitions =====================================
struct file_operations csi2tx_control_fops = {
    .read = device_read,
    .write = device_write,
    .unlocked_ioctl = nxp_csi2tx_control_ioctl,
    .open = device_open,
    .release = device_release, 
};

struct file_operations csi2tx_adc_fops = {
    .write = adc_write,
    .open = adc_open,
    .release = adc_release, 
};

//*************************** SysFs Attribute definitions ***************************************
static ssize_t pid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef DEBUG
    pr_info("[%s:%d] Read successful, PID Value =%d\n",__FILE__,__LINE__, pid);
#endif /* DEBUG */
    return sprintf(buf,"%d",pid);
}

static ssize_t pid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d",&pid);
#ifdef DEBUG
    pr_info("[%s:%d] PID(%x) is stored successfully\r\n",__FILE__,__LINE__,pid);
#endif /*DEBUG*/
    rcu_read_lock();
    ptask = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);
    if(ptask == NULL){
        pr_info(" [%s:%d] no such pid\n",__FILE__,__LINE__);
        rcu_read_unlock();
        return -1;
    }
    rcu_read_unlock();
    
    memset(&info, 0, sizeof(struct kernel_siginfo));
    info.si_signo = SIGUSR2;
    info.si_code = SI_QUEUE;    // this is bit of a trickery: SI_QUEUE is normally used by sigqueue from user space,

    return PAGE_SIZE;
}

#ifdef LOOP_BACK_TEST
/* Loop Back test */
static ssize_t loopback_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"[%s:%d] loopback_show Not implemented...\n",__FILE__,__LINE__);
}

static ssize_t loopback_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    uint32_t vc0_lines = 0;

    if (count > 0) {
        ret = sscanf(buf, "%x", &vc0_lines);
        if (ret >=1){ 
            iowrite32( 0x1, lbPtr + (0x00/4));
            iowrite32( vc0_lines, lbPtr + (0x40/4));
        }
        else {
            pr_alert("[%s:%d] Incorrect Frame Triggering\n",__FILE__,__LINE__);
        }
        return PAGE_SIZE;
    }
    else
    {
        pr_alert("[%s:%d] Copy From user Failed \n",__FILE__,__LINE__);
        return -1;
    }
}
#endif  //LOOP_BACK_TEST
/* USB SSD */
static ssize_t radarCycleCountShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{
    #ifdef DEBUG
    pr_info("[%s:%d] Read successful, radar Count =%d\n",__FILE__,__LINE__, radar_cycle_cnt);
    #endif
    return sprintf(resetBuf,"%d\n",radar_cycle_cnt);
}
static ssize_t radarCycleCountStore(struct device *dev, struct device_attribute *attr, const char *resetBuf, size_t count)
{
    sscanf(resetBuf, "%d",&radar_cycle_cnt);
    radarCycConst = radar_cycle_cnt;
    #ifdef DEBUG
    pr_info("[%s:%d] radar cycle count(%x) is stored successfully\r\n",__FILE__,__LINE__,radar_cycle_cnt);
    #endif /*DEBUG*/
    return PAGE_SIZE;
}
static ssize_t usbReadCompleteShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{
    #ifdef DEBUG
    pr_info("[%s:%d] Read successful, USB Read Complete =%d\n",__FILE__,__LINE__, usbReadComplete);
    #endif /*DEBUG*/
    return sprintf(resetBuf,"%d\n",usbReadComplete);
}

static ssize_t usbTransmitShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{
    #ifdef DEBUG
    pr_info("[%s:%d] Read successful, USB Cap =%d\n",__FILE__,__LINE__, usbCapEn);
    #endif /*DEBUG*/
    return sprintf(resetBuf,"%d\n",usbCapEn);
}
static ssize_t usbTransmitStore(struct device *dev, struct device_attribute *attr, const char *resetBuf, size_t count)
{
    int usbEnFlg;
    sscanf(resetBuf, "%d", &usbCapEn);
    usbEnFlg = usbCapEn;
    #ifdef DEBUG
        pr_info("[%s:%d]open usbCapEn=%d\n",__FILE__, __LINE__,usbCapEn);
    #endif /*DEBUG*/
    return PAGE_SIZE;
}

static ssize_t fpgaResetShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{
    #ifdef DEBUG
    pr_info("[%s:%d] Read successful, FPGAresetValue =%d\n",__FILE__,__LINE__, resetDone);
    #endif /*DEBUG*/
    return sprintf(resetBuf,"%d\n",resetDone);
}
static ssize_t fpgaResetStore(struct device *dev, struct device_attribute *attr, const char *resetBuf, size_t count)
{
    sscanf(resetBuf, "%d",&resetDone);
    atomic_set(&counter_th, 0);
    pr_info("[%s:%d] resetDone(%x) is stored successfully\r\n",__FILE__,__LINE__,resetDone);
    return PAGE_SIZE;
}
static ssize_t csi2txTransmitShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{    
    #ifdef DEBUG
    pr_info("[%s:%d]<Transmit status> FrameStatus= %d frameNumber=%d FrameSize =%d \n",__FILE__,__LINE__,FrameStatus, frameNumber, FrameSize );
    #endif /*DEBUG*/
    return sprintf(resetBuf,"%d %d %d\n",FrameStatus, frameNumber, FrameSize );
}

static DEVICE_ATTR(csi2txTransmitStatus, S_IRUGO , csi2txTransmitShow, NULL); 
static DEVICE_ATTR(radarCycleCountRegister, S_IWUSR | S_IRUGO , radarCycleCountShow, radarCycleCountStore);       //The USB Write/Or Failure Status
static DEVICE_ATTR(usbReadCompleteRegister, S_IRUGO, usbReadCompleteShow, NULL);       //The USB Write/Or Failure Status
static DEVICE_ATTR(usbTransmitEn, S_IWUSR | S_IRUGO, usbTransmitShow, usbTransmitStore);       //The capture has to be done to USB SSD
static DEVICE_ATTR(fpgaResetRegister, S_IWUSR | S_IRUGO, fpgaResetShow, fpgaResetStore);       //Write pid for process to be notified for interrupt

#ifdef LOOP_BACK_TEST
static DEVICE_ATTR(loopback, S_IWUSR | S_IRUGO, loopback_show, loopback_store);
#endif  //LOOP_BACK_TEST
static DEVICE_ATTR(register_pid, S_IWUSR | S_IRUGO , pid_show, pid_store);	//Write pid for process to be notified for interrupt
//================================= Device Constructors/Destructors ==================================================
static int csi2tx_construct_fpga_device(struct csi2tx_fpga_dev *dev, int minor, struct class *class)
{
    int err = 0;
    dev_t devno = MKDEV(csi2tx_dev_major, minor);
    struct device *device = NULL;

    BUG_ON(dev == NULL || class == NULL);

    mutex_init(&dev->csi2tx_mutex);

    cdev_init(&dev->cdev, &csi2tx_control_fops);
    dev->cdev.owner = THIS_MODULE;

    err = cdev_add(&dev->cdev, devno, 1);
    if (err)
    {
        pr_warn("[%s:%d] [target] Error %d while trying to add %s%d",__FILE__,__LINE__, err, "Custom PL control device %d", minor); 
        return err;
    }

    device = device_create(class, NULL, devno, NULL, "nxp_tx_control");

    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        pr_warn("[%s:%d] [target] Error %d while trying to create %s%d",__FILE__,__LINE__, err,  "Custom PL control device %d", minor-1);
        cdev_del(&dev->cdev);
        return err;
    }

    /* device attributes on sysfs */
    err = device_create_file(device, &dev_attr_register_pid);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_loopback);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_radarCycleCountRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    } 
    err = device_create_file(device, &dev_attr_usbReadCompleteRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_usbTransmitEn);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_fpgaResetRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_csi2txTransmitStatus);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }

    return 0;
}

static int csi2tx_construct_adc_device(struct csi2tx_adc_dev *dev, int minor, struct class *class)
{
    int err = 0;
    dev_t devno = MKDEV(csi2tx_dev_major, minor);
    struct device *device = NULL;

    BUG_ON(dev == NULL || class == NULL);

    if((minor - NUM_UUTS -1)%NUM_ADC_CHANNELS == 0){	//The frame struct is shared by every NUM_ADC_CHANNELS
        pcsi2tx_frame_devices[MY_ADC_FRAME(minor)].frame_data = NULL;     /* Memory is to be allocated when the device is opened the first time */
        pcsi2tx_frame_devices[MY_ADC_FRAME(minor)].frame_size = 0;
    }

    dev->my_frame = &pcsi2tx_frame_devices[MY_ADC_FRAME(minor)];	//Assign the correspondent shared structure
    dev->adc_code = (minor - NUM_UUTS -1)%NUM_ADC_CHANNELS; 	//Store channel number relative to uut

    mutex_init(&dev->csi2tx_mutex);

    cdev_init(&dev->cdev, &csi2tx_adc_fops);
    dev->cdev.owner = THIS_MODULE;

    err = cdev_add(&dev->cdev, devno, 1);
    if (err)
    {
        pr_warn("[%s:%d] [target] Error %d while trying to add %s%d",__FILE__,__LINE__, err, "adc_tx%d", (minor-NUM_UUTS)); 
        return err;
    }

    device = device_create(class, NULL, devno, NULL, "adc_tx%d", (minor-NUM_UUTS));

    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        pr_warn("[%s:%d] [target] Error %d while trying to create %s%d",__FILE__,__LINE__, err, "adc_tx%d", (minor-NUM_UUTS));
        cdev_del(&dev->cdev);
        return err;
    }

    return 0;
}

static void csi2tx_destroy_fpga_device(struct csi2tx_fpga_dev *dev, int minor, struct class *class)
{
    BUG_ON(dev == NULL || class == NULL);
    device_destroy(class, MKDEV(csi2tx_dev_major, minor));
    cdev_del(&dev->cdev);
    mutex_destroy(&dev->csi2tx_mutex);
    return;
}

static void csi2tx_destroy_adc_device(struct csi2tx_adc_dev *dev, int minor,struct class *class)
{
    BUG_ON(dev == NULL || class == NULL);
    cdev_del(&dev->cdev);
    device_destroy(class, MKDEV(csi2tx_dev_major, minor));
    mutex_destroy(&dev->csi2tx_mutex);
    return;
}
static void csi2tx_cleanup_module(int devices_to_destroy)
{
    int i;

    /* Get rid of character devices (if any exist) */
    if (devices_to_destroy == 0) goto out;
    csi2tx_destroy_fpga_device(&csi2tx_control_dev, 0, pcsi2tx_class);

    if (devices_to_destroy > 1){
        for(i = NUM_UUTS + 1  ; i<min(NUM_DEVICES, devices_to_destroy); i++){
            csi2tx_destroy_adc_device(&pcsi2tx_adc_devices[i-(NUM_UUTS +1  )], i, pcsi2tx_class);
        }
        kfree(pcsi2tx_frame_devices);
        kfree(pcsi2tx_adc_devices);
    }
out:
    if (pcsi2tx_class)
    {
        class_destroy(pcsi2tx_class);
    }
    unregister_chrdev_region(device_number, NUM_DEVICES);
    return;
}

//**************************** Module Init and Release ***************************************
static int __init nxp_csi2tx_driver_init(void)
{
    int err = 0;
    struct device_node *np = NULL;
    int devices_to_destroy = 0;
    int i = 0;

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);

    /*These are set to store and retrieve private data for the device. 
    One can store device specific structure here are retrieve it when device specific context is needed inside the device driver. 
    For example, all the private data structure for the device can be stored inside the structure 
    whose address is stored usig dev_set_drvdata() and retrieved using dev_get_drvdata().
    These are used only device drivers. */
    pcsi2tx_usb_dev = (struct nxp_datacap_usb_rd_wr_local *) kmalloc(sizeof(struct nxp_datacap_usb_rd_wr_local), GFP_KERNEL);
    if (!pcsi2tx_usb_dev) {
                pr_warn("Cound not allocate nxp-datacap-usb-rd-wr device\n");
                return -ENOMEM;
    }
    
    /* Get a range of minor numbers (starting with 0) to work with */
    err = alloc_chrdev_region(&device_number, 0, NUM_DEVICES, KBUILD_MODNAME);
    if (err < 0) {
        pr_warn("[%s:%d] [target] alloc_chrdev_region() failed\n",__FILE__,__LINE__);
        kfree(pcsi2tx_usb_dev);
        return err;
    }
    csi2tx_dev_major = MAJOR(device_number);

    /* Create device class (before allocation of the array of devices) */
    pcsi2tx_class = class_create(THIS_MODULE, KBUILD_MODNAME);
    if (IS_ERR(pcsi2tx_class)) {
        err = PTR_ERR(pcsi2tx_class);
        goto fail;
    }

    /* Allocate the array of frame devices, there are only NUM_UUTS frame devices even though there are NUM_UUTS * NUM_ADC_CHANNELS devices */
    pcsi2tx_frame_devices = (struct csi2tx_frame_dev *)kzalloc( NUM_UUTS * sizeof(struct csi2tx_frame_dev), GFP_KERNEL);
    if (pcsi2tx_frame_devices == NULL) {
        err = -ENOMEM;
        goto fail;
    }

    /* Allocate the array of frame devices, there are only NUM_UUTS frame devices even though there are NUM_UUTS * NUM_ADC_CHANNELS devices */
    pcsi2tx_adc_devices = (struct csi2tx_adc_dev *)kzalloc( NUM_UUTS * NUM_ADC_CHANNELS * sizeof(struct csi2tx_adc_dev), GFP_KERNEL);
    if (pcsi2tx_adc_devices == NULL) {
        err = -ENOMEM;
        goto fail;
    }

    /* Construct devices: 0 is generic controller, 1 to NUM_UUTS is  NUM_UUTS + 1 to NUM_UUTS + 4  is adc channels*/
    err = csi2tx_construct_fpga_device(&csi2tx_control_dev, 0, pcsi2tx_class);
    if (err) {
        devices_to_destroy = 1;
        goto fail;
    }

    for (i = NUM_UUTS + 1; i < NUM_DEVICES; i++){
        err = csi2tx_construct_adc_device(&pcsi2tx_adc_devices[i-(NUM_UUTS + 1)], i, pcsi2tx_class);
        if (err) {
            devices_to_destroy = i + 1;
            goto fail;
        }
    }

    //Map AXI addresses! 
    pframePtr =  ioremap(CSI2_TX_START_ADDR, (CSI2_TX_END_ADDR - CSI2_TX_START_ADDR) + 4 );
    if(pframePtr == NULL)
    {
        pr_alert("[%s:%d] Frame ptr is NULL and ioremap is failed .....\n",__FILE__,__LINE__);
        goto r_map;
    }


#ifdef LOOP_BACK_TEST
    lbPtr =  ioremap(CSI2_TX_SUBSYSTEM_START_ADDR , (CSI2_TX_SUBSYSTEM_END_ADDR -  CSI2_TX_SUBSYSTEM_END_ADDR ) +4 );
    if(lbPtr == NULL)
    {
        pr_alert("[%s:%d] Loop Back Ptr is NULL and ioremap is failed .....\n",__FILE__,__LINE__);
        goto r_map;
    }
#endif
    //Obtain Interrupt Numbers
    np = of_find_node_by_name(NULL,"nxp-csi2tx-driver"); // the <node_name> is the one before “@” sign in dtsi file.
    FRAME_INTERRUPT_TX = irq_of_parse_and_map(np,0);

    //Register Interrupt for Frame Grabber (frame ready)
    err = request_irq(FRAME_INTERRUPT_TX, data_isr, 0, KBUILD_MODNAME ".frameIsrTx", NULL);
    if (err < 0) {
        pr_alert("[%s:%d] request_irq %d for module %s failed with %d\n",__FILE__,__LINE__, FRAME_INTERRUPT_TX, KBUILD_MODNAME ".frameIsrTx", err);
        goto r_irq;
    }
    else { pr_info("[%s:%d] Frame Interrupt was succesfully registered: %d\n",__FILE__,__LINE__, FRAME_INTERRUPT_TX);}

    /*Creating work by Dynamic Method */
    INIT_WORK(&csi2tx_workq,workqueue_fn);

    /* task is initialized to NULL */
    ptask = NULL;

    atomic_set(&counter_th, 0);
    return 0; /* success */

r_irq:
    free_irq(FRAME_INTERRUPT_TX, NULL);
r_map:
    iounmap(pframePtr);
#ifdef LOOP_BACK_TEST
    iounmap(lbPtr);
#endif
fail:
    csi2tx_cleanup_module(devices_to_destroy);
    if(pcsi2tx_usb_dev) 
        kfree(pcsi2tx_usb_dev);
    return err;
}

static void __exit nxp_csi2tx_driver_exit(void)
{
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    iounmap(pframePtr);
#ifdef LOOP_BACK_TEST
    iounmap(lbPtr);
#endif
    free_irq(FRAME_INTERRUPT_TX, NULL);

    csi2tx_cleanup_module(NUM_DEVICES);
    if(pcsi2tx_usb_dev) 
        kfree(pcsi2tx_usb_dev);
    if (gp_usbfp != NULL)
    {
        pr_info("Freeing the path of SSD and closing the Device(SSD) File \n");
        if (file_count(gp_usbfp))
        {
            filp_close(gp_usbfp, NULL);
            gp_usbfp = NULL;
        }
    }
}

module_init(nxp_csi2tx_driver_init);
module_exit(nxp_csi2tx_driver_exit);

//********************************** Licensing ***************************************************
MODULE_DESCRIPTION("NXP driver to control the custom PL block for csi2tx devices communication and data capture");
MODULE_LICENSE("GPL v2");


