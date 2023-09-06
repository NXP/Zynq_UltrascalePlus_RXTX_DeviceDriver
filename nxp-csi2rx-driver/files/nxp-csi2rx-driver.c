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

#include "nxp-csi2rx-driver.h"

#define NUM_UUTS  1								//Max number of supported UUTs
#define NUM_ADC_CHANNELS 4						//Number of ADC channels per UUT
#define NUM_DEVICES (1 + NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS))	//Each uut will have a adc device and a frame device (do we need a status device? for interrupts and so on ...)
#define BUF_LEN 5000 							// Max length of the message from the device
#define SUCCESS 0								//Success code
#define TRUE 1
#define FALSE 0
#define MY_ADC_FRAME(I) ((I - NUM_UUTS -1)/NUM_ADC_CHANNELS)		//Macro to find out the corresponding frame device from adc device minor 
#define MAX_RADAR_CYCLE_COUNT 0xFFFFFFFF
#ifdef DEBUG_APPS 
#define MAX_FILE_SIZE       512
#endif /*DEBUG_APPS*/
//***************** Device structures ********************
struct csi2rx_fpga_dev {
    struct mutex csi2rx_mutex; 
    struct cdev cdev;
};
struct csi2rx_frame_dev {
    uint64_t *frame_data;
    uint32_t frame_address;
    uint32_t frame_size;
    uint8_t  BypassOrDecimated;
};
struct csi2rx_adc_dev{
    struct csi2rx_frame_dev *my_frame;
    struct mutex csi2rx_mutex; 
    struct cdev cdev;
    uint8_t adc_code;
};

struct nxp_datacap_usb_rd_wr_local {
        struct work_struct work;        /* used in workqueue */
        uint32_t frame_address;
        uint32_t frame_size;
        uint8_t  BypassOrDecimated;
        struct device   *dev_ptr;
};
#ifdef TIME_STAMP_EN
static atomic_t counter_bh,counter_irq;
#endif
static atomic_t counter_th;
static int resetDone = 0; // Stores application FPGA resetDone in user space
static int usbCapEn  = 0; // To Capture to USB SSD the files
static int usbWriteComplete  = 0; // To Inform the Status of USB Write Complete 
static uint32_t radar_cycle_count = 0;
static uint32_t totalFrameNumber = 0;
static uint32_t radarCycConst = 0;
static uint32_t tm_year = 0;
static uint32_t tm_mday = 0;
static uint32_t tm_mon  = 0;
static uint32_t tm_hour = 0;
static uint32_t tm_min  = 0;
static uint32_t tm_sec  = 0;


//================== Device Variables ========================
static dev_t csi2rx_dev_major , device_number;									//Device identifier
static struct csi2rx_frame_dev *csi2rx_frame_devices = NULL;		//Collection of Frame devices used by ADC devices
static struct csi2rx_adc_dev *csi2rx_adc_devices = NULL;			//Collection of ADC devices
static struct csi2rx_fpga_dev csi2rx_control_dev;					//Single instance of custom PL controller
static struct class *csi2rx_class = NULL;						//Driver class
//******************* Interrupt codes *********************
static int FRAME_INTERRUPT_RX = 0;					// Place to store OS interrupt code for SPI HW int	
//=================== Kernel operation variables ===================
static unsigned int csTimestamps[100] = {0};		// Storage of event timestamps
static char msg[BUF_LEN]; /* The msg the device will give when asked */
static char *msg_Ptr;
static int captureCounter = 0;
static int frameSize = 10000000;
static int frameOffset = 0;
static unsigned int *framePtr;

static unsigned int *MixelFramePtr; /*Mixel Frame pointer */
static unsigned int detectedChirps;
static struct task_struct *t = NULL;
static struct kernel_siginfo info;
static bool mixelEnable = false;
struct work_struct csi2_work;        /* used in workqueue */
struct device   *gdevice =  NULL;
struct nxp_datacap_usb_rd_wr_local *csi2rx_usb_dev = NULL;
static const char *path = NULL;
static const char *capType = "Norm"; 
static uint32_t samples=0,chirps=0;
static uint32_t totalBytesWritten = 0;
static struct file *g_cfile = NULL;
#ifdef DEBUG_APPS 
static char filename[MAX_FILE_SIZE]={0};
#endif /*DEBUG_APPS*/
static void workq_fun(struct work_struct *w_arg)
{
	uint32_t frameSizeBytes = 0;
        uint64_t *frame_data=NULL;
        ssize_t nbytes;
#ifdef TIME_STAMP_EN
        uint64_t now_timestamp,before_timestamp;
#endif
        uint32_t count=0;
	uint8_t const *ptr = NULL;
        int ret;


	if(csi2rx_usb_dev->BypassOrDecimated) {
                frameSizeBytes = ((csi2rx_usb_dev->frame_size / 4) * 8) / 2; //4 lates to 8 bytes, but we take a fourth as we only read for one adc at a time
        }
        else
        {
                frameSizeBytes = ((csi2rx_usb_dev->frame_size / 4) * 8) / 4; //lates to 8 bytes, but we take a fourth as we only read for one adc at a time
        }

        if(csi2rx_usb_dev->BypassOrDecimated) {
                frame_data = (uint64_t *)memremap(csi2rx_usb_dev->frame_address, (frameSizeBytes *2), MEMREMAP_WB); //Each 64 bit long (8 bytes) contains 4 samples
        }
        else
        {
                frame_data = (uint64_t *)memremap(csi2rx_usb_dev->frame_address, (frameSizeBytes *4), MEMREMAP_WB); //Each 64 bit long (8 bytes) contains 4 samples
        }
	 if (frame_data  == NULL) {
                pr_err("Could not remap frame memory at 0x%x for %d samples!\n", csi2rx_usb_dev->frame_address, frameSizeBytes);
		if(t == NULL)
                {
                        pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
                }
                else
                {
			/* 0x01  Memory allocation failed.*/
                        /* send the signal */
                        info.si_int = 2;     //real time signals may have 32 bits of data.
                        ret = send_sig_info(SIGUSR1, &info, t);    //send the signal
                        if (ret < 0) {
                                pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
                        }
                }
		usbWriteComplete = FALSE;
		radar_cycle_count = 0;
		totalFrameNumber = 0;
                return;
       }
       if(g_cfile == NULL)
       {
               pr_err("[%s:%d] Error g_cfile is NULL \n",__FILE__,__LINE__);
	       return;
       }
#ifdef TIME_STAMP_EN
        before_timestamp = ktime_get_real_ns();
#endif
        count = (csi2rx_usb_dev->frame_size *2 );
#ifdef DEBUG
//        printk("Count %d\n",count);
#endif
        ptr = ((const void *)((uint8_t*)(frame_data )));
        while (count > 0) {
                nbytes = kernel_write(g_cfile, ptr, count, &g_cfile->f_pos);
		if(nbytes == 0)
		{
			memunmap(frame_data);
                        filp_close(g_cfile, NULL);
			kfree(path);
		        if(t == NULL)
                        {
                                pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
                        }
                        else
                        {
				/* 0x05	Input/Output Error During Writes */
                                /* send the signal */
                                info.si_int = 5;     //real time signals may have 32 bits of data.
                                ret = send_sig_info(SIGUSR1, &info, t);    //send the signal
                                if (ret < 0) {
                                        pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
                                }
                        }
			usbWriteComplete = FALSE;
			pr_err(">>>>> IO Error totalFrameNumber %d radar_cycle_count %d \n",totalFrameNumber,radar_cycle_count);
	    		radar_cycle_count = 0;
			totalFrameNumber = 0;
			return;
		}
                if (nbytes < 0)
                {
                        memunmap(frame_data);
                        filp_close(g_cfile, NULL);
			kfree(path);
                        if(t == NULL)
                        {
                                pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
                        }
                        else
                        {
				/* 0x04	Error in Write to File (Write Failed) / No space left on device.*/
                                /* send the signal */
                                info.si_int = 4;     //real time signals may have 32 bits of data.
                                ret = send_sig_info(SIGUSR1, &info, t);    //send the signal
                                if (ret < 0) {
                                        pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
                                }
                        }
                        pr_err(">>>>> Error in write /  No space left on device totalFrameNumber %d radar_cycle_count %d \n",totalFrameNumber,radar_cycle_count);
			usbWriteComplete = FALSE;
	    		radar_cycle_count = 0;
			totalFrameNumber = 0;
                        return;
                }
                count -= nbytes;
                ptr += nbytes;
        }

	totalBytesWritten = g_cfile->f_pos;
        if(t == NULL)
        {
                pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
        }
        else
        {
		/*Ok (Write Complete)*/
                /* send the signal */
                info.si_int = 6;     //real time signals may have 32 bits of data.
                ret = send_sig_info(SIGUSR1, &info, t);    //send the signal
                if (ret < 0) {
                        pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
                }
        }
        usbWriteComplete =  TRUE; //Done Success
	
#ifdef TIME_STAMP_EN
        now_timestamp = ktime_get_real_ns();
        atomic_inc(&counter_bh);
	if(atomic_read(&counter_bh) != atomic_read(&counter_irq))
	{
        	pr_info("In BH: counter_irq = %d, counter_bh = %d, \n", atomic_read(&counter_irq), atomic_read(&counter_bh));
                pr_info("<<<< >>> wrote nbytes = %d Time taken in nanoseconds: %llu\n",nbytes,(now_timestamp - before_timestamp));
	}
#endif
	if(radar_cycle_count == 0x0)
	{
        	if (file_count(g_cfile)) {
#ifdef DEBUG
        		pr_info("closing the Device(SSD) File (wq) \n");
#endif
                	filp_close(g_cfile, NULL);
		        g_cfile = NULL;
        	}
	}
        memunmap(frame_data);
	return;
}

//====================== Custom PL Control Device Functions ========================
static int device_open(struct inode *inode, struct file *filp){
    unsigned int mj = imajor(inode);
    unsigned int mn = iminor(inode);
    int i;

    struct csi2rx_fpga_dev *dev = &csi2rx_control_dev;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    if (mj != csi2rx_dev_major || mn != 0)
    {
        pr_warn("[%s:%d] [target] " "No device found with minor=%d and major=%d\n",__FILE__,__LINE__, mj, mn);
        return -ENODEV; /* No such device */
    }

    /* store a pointer to struct cfake_dev here for other methods */
    filp->private_data = &csi2rx_control_dev; 

    if (inode->i_cdev != &dev->cdev)
    {
        pr_warn("[%s:%d] [target] open: internal error\n",__FILE__,__LINE__);
        return -ENODEV; /* No such device */
    }

    sprintf(msg, "Detected %d Chirps:\n", detectedChirps);
    for(i = 0; i<detectedChirps; i++){
        sprintf(msg + strlen(msg),"\tChirp %d pulse duration: %d.%03dus\n", i + 1, csTimestamps[i]/1000, csTimestamps[i]%1000 );
    }
    detectedChirps = 0;
    msg_Ptr = msg;

    try_module_get(THIS_MODULE);
    return SUCCESS;
}

static int device_release(struct inode *inode, struct file *file){
    module_put(THIS_MODULE);
    return SUCCESS;
}

static ssize_t device_read(struct file *filp, char __user * buffer, size_t length, loff_t * offset){
    int bytes_read;

    bytes_read = 0;
    if (*offset >= strlen(msg)) return 0; /* EOF */
    if (*offset + length > strlen(msg)) length = strlen(msg) - *offset;

    if (*msg_Ptr == 0) return 0;

    while (length && *msg_Ptr) {
        put_user(*(msg_Ptr++), buffer++);
        length--;
        bytes_read++;
    }
    *offset += bytes_read;
    return bytes_read;
}

static ssize_t device_write(struct file *filp, const char __user * buffer, size_t length, loff_t * offset){
    struct csi2rx_fpga_dev *dev;

    dev = (struct csi2rx_fpga_dev *)filp->private_data;

    return length;
}


//============================== FRAME Device Functions ==============================
static int adc_open(struct inode *inode, struct file *filp){
    unsigned int mj = imajor(inode);
    unsigned int mn = iminor(inode);

    struct csi2rx_adc_dev *dev = NULL;
#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */

    if (mj != csi2rx_dev_major || mn <= NUM_UUTS || mn > (NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS)))
    {
        pr_warn("[%s:%d] [target] " "No device found with minor=%d and major=%d\n",__FILE__,__LINE__, mj, mn);
        return -ENODEV; /* No such device */
    }

    /* store a pointer to struct cfake_dev here for other methods */
    dev = &csi2rx_adc_devices[mn - (NUM_UUTS + 1)];
    filp->private_data = dev; 

    if (inode->i_cdev != &dev->cdev)
    {
        pr_warn("[%s:%d] [target] open: internal error\n",__FILE__,__LINE__);
        return -ENODEV; /* No such device */
    }

    return 0;
}

static int adc_release(struct inode *inode, struct file *filp){
#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    return 0;
}
static ssize_t adc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos){
    struct csi2rx_adc_dev *dev;
    struct csi2rx_frame_dev *frame_dev;
    ssize_t retval;
    int i;
    uint32_t frameSizeBytes;
    unsigned short *adc = NULL;

    dev         =   (struct csi2rx_adc_dev *)filp->private_data;
    frame_dev   =   (struct csi2rx_frame_dev *)dev->my_frame;
    retval      =   0;
    i           =   0;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */

    if(frame_dev-> BypassOrDecimated) {

        frameSizeBytes = ((frame_dev->frame_size / 4) * 8) / 2; //4 samples in one 64 long long, each of those translates to 8 bytes, but we take a fourth as we only read for one adc at a time
    }
    else
    {
        frameSizeBytes = ((frame_dev->frame_size / 4) * 8) / 4; //4 samples in one 64 long long, each of those translates to 8 bytes, but we take a fourth as we only read for one adc at a time
    }
    
    if (mutex_lock_killable(&dev->csi2rx_mutex)) {
        printk(KERN_EMERG"Inside %s before -EINTR\n ",__FUNCTION__);
        return -EINTR;
    }
    if (*f_pos >= frameSizeBytes) {
        printk(KERN_EMERG"Samples collected per ADC is = %d\n", (frameSizeBytes >> 1));
        goto out; /* EOF */
    }
    if ((*f_pos + count) > frameSizeBytes) {
        count = frameSizeBytes - *f_pos;
        printk(KERN_EMERG"Inside %s frameSizeBytes = %u count =%lu\n ",__FUNCTION__,frameSizeBytes,count); 
    }		

    if(frame_dev-> BypassOrDecimated) {
        frame_dev->frame_data = (uint64_t *)memremap(frame_dev->frame_address, (frameSizeBytes *2), MEMREMAP_WB); //Each 64 bit long (8 bytes) contains 4 samples
    }
    else
    {
        frame_dev->frame_data = (uint64_t *)memremap(frame_dev->frame_address, (frameSizeBytes *4), MEMREMAP_WB); //Each 64 bit long (8 bytes) contains 4 samples
    }

    if (frame_dev->frame_data  == NULL) {
        printk(KERN_ALERT "Could not remap frame memory at 0x%x for %d samples!\n", frame_dev->frame_address, frameSizeBytes);
        goto out;
    }
    adc = (uint16_t *)vmalloc(frameSizeBytes);
    if(!adc)
    {
        printk(KERN_ALERT "Vmalloc Failed>>>>>>>>>>>>\n");
        memunmap(frame_dev->frame_data);
        goto out;
    }

    if(frame_dev-> BypassOrDecimated) {
        for (i=0; i< frame_dev->frame_size/4; i+=2) {
            *(adc + (i *2))     = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 3);		//S1 ADCx
            *(adc + (i *2) + 1) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 2);		//S2 ADCx
            *(adc + (i *2) + 2) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 1);		//S3 ADCx
            *(adc + (i *2) + 3) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 0);		//S4 ADCx
        }
    }
    else {  
        for (i=0; i< frame_dev->frame_size/4; i+=4) {
            *(adc + i)     = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 3);      //S1 ADCx
            *(adc + i + 1) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 2);		//S2 ADCx
            *(adc + i + 2) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 1);		//S3 ADCx
            *(adc + i + 3) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 0);		//S4 ADCx
        }
    }
    if (copy_to_user((void __user*)buf, ((const void *)((uint8_t*)(&adc[0]) + *f_pos)), count) != 0)
    {
        retval = -EFAULT;
        goto out1;
    }
    //printk(KERN_ALERT "END of the copy to user loop\n");
    *f_pos += count;
    retval = count;

    // printk(KERN_EMERG"Exit %s f_pos = %d count = %d Frame_Buffer = %d loop count = %d \n ",__FUNCTION__,*f_pos, count,frameSizeBytes,i);
out1:	
    vfree(adc);
    memunmap(frame_dev->frame_data);
out:
    mutex_unlock(&dev->csi2rx_mutex);
    return retval;

}

//************************************* HW IO ISRs ****************************************
static irqreturn_t data_isr(int irq,void*dev_id) {      
    int i = 0, ret = 0;
    int captureBuffer = 0;
    int EnableLonCap = 0;
    int Enable_80msps = 0;
    uint32_t irqRet = 0 ,mixelIrqRet = 0;
#ifdef DEBUG
    pr_info("[%s:%d]  calling func %s \n",__FILE__,__LINE__, __func__);
#endif
    captureBuffer = ioread32(framePtr + (0x2B00/4));    //Get the buffer address
    EnableLonCap  = ioread32(framePtr + (0x0300/4)); 
    Enable_80msps = ioread32(framePtr + (0x0C00/4));
    
    //Workaround 
    if(Enable_80msps & 0xDEADBEEF)
    {
        //pr_info("[%s:%d] Resetting Enable_80msps value to zero \n",__FILE__,__LINE__);
        Enable_80msps = 0x0;
    }

    //Read the pointer information to know which devices have a frame captured
    switch (captureBuffer){
        case 2:
            frameOffset = 0x4000000;
            break;
        case 3:
            frameOffset = 0x8000000;
            break;
        case 0:
            frameOffset = 0xC000000;
            break;
        default:
            frameOffset = 0x0;
            break;
    }

    frameSize =  ioread32(framePtr + (CSI2RX_FRAME_LEN/4));	//Get value from FPGA register;
    
    irqRet = ioread32(framePtr + (CSI2RX_IRQ_STATUS/4));
    if (irqRet != 0x2) {
        pr_alert("[%s:%d] Error During Frame Capture. IRQ: %d!!!\n",__FILE__,__LINE__, irqRet);
        goto out;
    }
    mixelIrqRet = ioread32(MixelFramePtr + ( RX_IRQ_STATUS_OFFSET/4));
    if (mixelIrqRet & 0x1FFF) {
    	//pr_info("[%s:%d]  Mixel IRQ status %X \n",__FILE__,__LINE__, (mixelIrqRet & 0x1FFF));
	}

    if(((EnableLonCap & 0x00000002) >> 1) == 1) {
	capType = "Long";
        for (i=0; i<NUM_UUTS;i++) {
            csi2rx_frame_devices[i].frame_address = 0x40000000 + (0x10000000 * i);
            csi2rx_frame_devices[i].BypassOrDecimated = ((Enable_80msps & 0x00000008) >> 3);
            csi2rx_frame_devices[i].frame_size = frameSize;
        }
    }
    else {
	capType = "Norm";
        for (i=0; i<NUM_UUTS;i++) {
            csi2rx_frame_devices[i].frame_address = 0x40000000 + (0x10000000 * i) + frameOffset;
            csi2rx_frame_devices[i].BypassOrDecimated = ((Enable_80msps & 0x00000008) >> 3);
            csi2rx_frame_devices[i].frame_size = frameSize;
        }
    }

    if(t == NULL)
    {
        pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
    }
    else
    {
        /* send the signal */
        info.si_int = 0;     //real time signals may have 32 bits of data.
        ret = send_sig_info(SIGUSR1, &info, t);    //send the signal
        if (ret < 0) {
            pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
            return IRQ_HANDLED;
        }
    }
    if (usbCapEn)
    {
#ifdef DEBUG
        // printk("Scheduling the work_queue radar_cycle_count %d\n",radar_cycle_count);
#endif
        csi2rx_usb_dev->frame_address = csi2rx_frame_devices[0].frame_address;
        csi2rx_usb_dev->BypassOrDecimated = csi2rx_frame_devices[0].BypassOrDecimated;
        csi2rx_usb_dev->frame_size = frameSize;
        if (radar_cycle_count != 0)
        {
            schedule_work(&csi2_work);
#ifdef TIME_STAMP_EN
            atomic_inc(&counter_irq);
#endif
            // clear interrupt
            // start csi capture
            if ((radar_cycle_count-1) != 0)
            {
                iowrite32(0xFF, framePtr + (CSI2RX_IRQ_CLR / 4));
                iowrite32(0x1, framePtr + (CSI2RX_FRAME_START / 4));
            }
            // decrement radar_cycle_count
            radar_cycle_count--;
	    totalFrameNumber++;
        }
        else{
            usbCapEn = 0;
        }
    }
out:
    captureCounter++;
    return IRQ_HANDLED;
}
/****************************** IOCTL Functions **************************/
void ioctl_csi2rx_irq_reset( void )
{
        iowrite32( 0xFF, framePtr + (CSI2RX_IRQ_CLR/4));
}
void ioctl_csi2rx_trigger(void)
{
        iowrite32( 0xFF, framePtr + (CSI2RX_IRQ_CLR/4));
        iowrite32( 0x1, framePtr + (CSI2RX_FRAME_START/4));
}
void ioctl_csi2rx_frame_start(void)
{
        iowrite32( 0x1, framePtr + (CSI2RX_FRAME_START/4));
}

void csi2rx_config_reg_write(struct csi2_rx_control_reg csi2_rx)
{
    iowrite32( csi2_rx.Nsample, framePtr + ( CSI2RX_SAM_PER_FRAME/4));
    iowrite32( csi2_rx.Ncycle, framePtr + ( CSI2RX_N_CYCLE/4));
    iowrite32( csi2_rx.Nchirps1, framePtr + ( CSI2RX_N_CHIRPS1/4));
    iowrite32( csi2_rx.Nchirps2, framePtr + ( CSI2RX_N_CHIRPS2/4));
    iowrite32( csi2_rx.Nchirps3, framePtr + ( CSI2RX_N_CHIRPS3/4));
    iowrite32( csi2_rx.Nchirps4, framePtr + ( CSI2RX_N_CHIRPS4/4));
    iowrite32( csi2_rx.Rxgen , framePtr + ( CSI2RX_RXGEN_SIZE/4));
    iowrite32( csi2_rx.TestData1 , framePtr + (CSI2RX_TEST_DATA_CH1/4));
    iowrite32( csi2_rx.TestData2 , framePtr + (CSI2RX_TEST_DATA_CH2/4));
    iowrite32( csi2_rx.TestData3 , framePtr + (CSI2RX_TEST_DATA_CH3/4));
    iowrite32( csi2_rx.TestData4 , framePtr + (CSI2RX_TEST_DATA_CH4/4));
}


static int32_t ioctl_create_metadata(struct metadata_header mhdr)
{
    struct rtc_time tm;
    int err = 0;
    struct timespec64 time;
    unsigned long local_time;
    ssize_t nbytes;

    if(mhdr.capture)
    {
        capType = "Long";
#ifdef DEBUG
	pr_info("[%s:%d] Long Capture.\n",__FILE__,__LINE__);
#endif
    }
    else
    {
	capType = "Norm";
#ifdef DEBUG
	pr_info("[%s:%d] Normal Capture.\n",__FILE__,__LINE__);
#endif 
    }
    atomic_inc(&counter_th);

    ktime_get_ts64(&time);
    local_time = (u32)(ktime_get_real_seconds() - (sys_tz.tz_minuteswest * 60));
    rtc_time64_to_tm(local_time,&tm);

    tm_year = tm.tm_year + 1900;
    tm_mday = tm.tm_mday;
    tm_mon  = tm.tm_mon + 1;
    tm_hour = tm.tm_hour;
    tm_min  = tm.tm_min;
    tm_sec  = tm.tm_sec;

    samples = mhdr.numOfSamples_1;
    chirps =  mhdr.numOfChirps_1;
    if(g_cfile != NULL)
    {
	if (file_count(g_cfile)) {
		filp_close(g_cfile, NULL);
		g_cfile = NULL;
	}
    }
    if(path != NULL)
    {
        kfree(path);
    }

    path = kasprintf(GFP_KERNEL, "/run/media/sda1/Cap%s%d_%dS_%dC_%uRC_%04d%02d%02d-%02d-%02d-%02d-%d.bin", capType,atomic_read(&counter_th),samples,chirps,radarCycConst,tm_year,tm_mday,tm_mon,tm_hour,tm_min,tm_sec,jiffies_to_msecs(jiffies));
     if(!path)
     {
          printk("Error on Path >>>>>>>>>\n");
	  if(t == NULL)
          {
               pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
          }
          else
          {
		  /* 0x02 Error in the USB SSD Device Path (Out of Resources)*/
                 /* send the signal */
                 info.si_int = 2;     //real time signals may have 32 bits of data.
                 err = send_sig_info(SIGUSR1, &info, t);    //send the signal
                 if (err < 0) {
                          pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
                  }
          }
          pr_err(" Error in the USB SSD Device Path \n");
	  radar_cycle_count = FALSE;
	  totalFrameNumber = 0;
          return 0;
     }
     usbWriteComplete  = 0;
#ifdef DEBUG_APPS 
     memset(filename,0,MAX_FILE_SIZE);
     memcpy(filename,path,MAX_FILE_SIZE);
#endif /*DEBUG_APPS*/
     g_cfile = filp_open(path, O_CREAT | O_WRONLY | O_APPEND | O_LARGEFILE | O_DSYNC, S_IWUSR | S_IRUSR);
     if (IS_ERR(g_cfile)) {
            if(t == NULL)
            {
                pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
            }
            else
            {
		/* 0x03 Error in Open of Device Path(Open Failed) */
                /* send the signal */
                info.si_int = 3;     //real time signals may have 32 bits of data.
                err = send_sig_info(SIGUSR1, &info, t);    //send the signal
                if (err < 0) {
                        pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
                }
            }
            pr_err("filed to open (%s) path \n", path);
            kfree(path);
	    radar_cycle_count = FALSE;
	    totalFrameNumber = 0;
            return 0;
     }
     nbytes = kernel_write(g_cfile, &mhdr, sizeof(struct metadata_header), &g_cfile->f_pos);
     if(nbytes < 0)
     {
	   pr_err("Kernel Write Failed\n");
	   if(t == NULL)
           {
                 pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
           }
           else
           {
               /* 0x04 Error in Write to File (Write Failed) / No space left on device.*/
               /* send the signal */
               info.si_int = 4;     //real time signals may have 32 bits of data.
               err = send_sig_info(SIGUSR1, &info, t);    //send the signal
               if (err < 0) {
                     pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
               }
           }

           usbWriteComplete =  FALSE; //USB Write Failed
	   radar_cycle_count = FALSE;
	   totalFrameNumber = 0;
           filp_close(g_cfile, NULL);
	   g_cfile = NULL;
           kfree(path);
	   //return nbytes;
	   return 0;
     }
     else if(nbytes == 0)
     {
	   pr_err("Kernel Write Failed IO Error\n");
	   if(t == NULL)
           {
               pr_info("[%s:%d]  RX Task is NULL \n",__FILE__,__LINE__);
           }
           else
           {
               /* 0x05 Input/Output Error During Writes */
               /* send the signal */
               info.si_int = 5;     //real time signals may have 32 bits of data.
               err = send_sig_info(SIGUSR1, &info, t);    //send the signal
               if (err < 0) {
                      pr_info("[%s:%d] Error sending signal \n",__FILE__,__LINE__);
               }
           }
           usbWriteComplete =  FALSE; //USB Write Failed
	   radar_cycle_count = FALSE;
	   totalFrameNumber = 0;
           filp_close(g_cfile, NULL);
	   g_cfile = NULL;
           kfree(path);
	   return 0;
     }

     totalBytesWritten = g_cfile->f_pos;
     usbWriteComplete =  TRUE; //Done Success
#ifdef DEBUG
     pr_info("usnWriteComplete Done! metaData Header write is Success \n");
     pr_info("SSD  path >>>>>>>>>>>>>>>>  (%s)  \n", path);
#endif
   return err;
}

static long nxp_csi2rx_control_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    uint32_t flength;
    struct csi2_rx_control_reg csi2_rx;
    struct metadata_header metaHdr;
    int err;
    switch(cmd) {
        case  IOCTL_REG_WRITE_RX:
            copy_from_user(&csi2_rx, (struct csi2_rx_control_reg *)arg, sizeof(struct csi2_rx_control_reg));
            csi2rx_config_reg_write(csi2_rx );
            csi2_rx.bconf = true;
            copy_to_user((struct csi2_rx_control_reg*)arg, &csi2_rx,sizeof(struct csi2_rx_control_reg));
            break;
        case IOCTL_FRAME_START_RX:
            ioctl_csi2rx_frame_start();
            break;
        case IOCTL_FRAME_LEN_RX:
            flength = ioread32(framePtr +(0x1000/4));
            copy_to_user((uint32_t*)arg, &flength,sizeof(uint32_t));
            break;
        case IOCTL_IRQ_READ_RX:
            copy_from_user(&csi2_rx, (struct csi2_rx_control_reg *)arg, sizeof(struct csi2_rx_control_reg));
            csi2_rx.irq_stat = ioread32(framePtr +(0x2C00/4));
            copy_to_user((struct csi2_rx_control_reg*)arg, &csi2_rx, sizeof( struct csi2_rx_control_reg ));
            break;
        case IOCTL_IRQ_RESET_RX :
            ioctl_csi2rx_irq_reset();
            break;
        case IOCTL_FPGA_TRIGGER_RX :
            ioctl_csi2rx_trigger();
            break;
        case IOCTL_REG_PTRN_WRITE_RX :
            copy_from_user(&csi2_rx, (struct csi2_rx_control_reg *)arg, sizeof(struct csi2_rx_control_reg));
            iowrite32( csi2_rx.patternID, framePtr + ( CSI2RX_RXGEN_CTL_PATTERN/4));
            break;
        case IOCTL_METADATA_HEADER:
	         if(copy_from_user(&metaHdr, (struct metadata_header *)arg, sizeof(struct metadata_header)))
		         return -EFAULT;
	         err = ioctl_create_metadata(metaHdr);
	         if(!err)
		         return 0;
	         else
		         return err;
        default:
            pr_info("[%s:%d] Unknown IOCTL ...\n",__FILE__,__LINE__);
            return -EINVAL;
    }
    return 0;
}

//============================ File Operation Definitions =====================================
struct file_operations csi2rx_control_fops = {
    .read = device_read,
    .write = device_write,
    .unlocked_ioctl = nxp_csi2rx_control_ioctl,
    .open = device_open,
    .release = device_release, /* a.k.a. close */
};

struct file_operations csi2rx_adc_fops = {
    .read = adc_read,
    .open = adc_open,
    .release = adc_release, /* a.k.a. close */
};

//*************************** SysFs Attribute definitions ***************************************
static ssize_t pid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef DEBUG
    pr_info("[%s:%d] Read successful, PID Value =%d\n",__FILE__,__LINE__, pid);
#endif 
    return sprintf(buf,"%d", pid);
}

static ssize_t pid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d",&pid);
#ifdef DEBUG
    pr_info("[%s:%d] PID(%x) is stored successfully\r\n",__FILE__,__LINE__,pid);
#endif
    rcu_read_lock();
    t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);
    if(t == NULL){
        printk("no such pid\n");
        rcu_read_unlock();
        return -1;
    }
    rcu_read_unlock();

    memset(&info, 0, sizeof(struct kernel_siginfo));
    info.si_signo = SIGUSR1;
    info.si_code = SI_QUEUE;    // this is bit of a trickery: SI_QUEUE is normally used by sigqueue from user space,
    
    return PAGE_SIZE;
}

#ifdef UNUSED_ATTR
static ssize_t capture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "Not implemented\n");
}

static ssize_t capture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    int numChirps;
    int numSamples;

    numChirps = 0;
    numSamples = 0;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    if (count > 0) {
        ret = sscanf(buf, "%d %d", &numChirps, &numSamples);
        if (ret >=1){ //Check that either the number of chirps or the number of samples is passed
            iowrite32(0x3, framePtr + (0x2D00/4)); 			//Reset Interrupts
            //iowrite32(0x1, framePtr + (0x0C00/4)); 			//Configure Polarity
            iowrite32(0x0, framePtr + (0x0300/4)); 			//Disable Test Mode
            iowrite32(numChirps, framePtr + (0x2100/4)); 	        //Nchirps
            iowrite32(numSamples, framePtr + (0x0800/4)); 	        //Nchirps
            //iowrite32(0x1, framePtr + (0x0400/4)); 			//Trigger
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

static ssize_t func_80_msps_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "functional_80msps_capture_show Not implemented\n");
}

static ssize_t func_80_msps_cap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    int numSamples =0;
    int numChirps;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    if (count > 0) {
        ret = sscanf(buf, "%d %d", &numChirps, &numSamples);
        if (ret >=1){ //Check that the number of samples is passed
            iowrite32(0x3, framePtr + (0x2D00/4)); 	       //Reset Interrupts
            //iowrite32(0x9, framePtr + (0x0C00/4)); 	        //Configure Polarity and packat data format
            iowrite32(0x0, framePtr + (0x0300/4)); 		//Disbale TestMode
            iowrite32(numChirps, framePtr + (0x2100/4)); 	//Nchirps
            iowrite32(numSamples, framePtr + (0x0800/4)); 	//Nchirps
            //iowrite32(0x1, framePtr + (0x0400/4)); 	        //Trigger
        }
        else {
            pr_alert("[%s:%d] Incorrect Frame Triggering\n",__FILE__,__LINE__);
        }
        return PAGE_SIZE;
    }

    else{
        pr_alert("[%s:%d] Copy From user Failed \n",__FILE__,__LINE__);
        return -1;
    }
}

static ssize_t long_80_msps_cap_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
    return snprintf(buf, PAGE_SIZE, "long_80msps_capture_show Not implemented\n");
}

static ssize_t long_80_msps_cap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    int numSamples = 0;
    int numChirps;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    if (count > 0) {
        ret = sscanf(buf, "%d %d", &numChirps, &numSamples);
        if (ret >=1){ //Check that number of samples is passed
            iowrite32(0x3, framePtr + (0x2D00/4)); 			//Reset Interrupts
            iowrite32(0x9, framePtr + (0x0C00/4)); 		        //Configure Polarity and packat data format
            iowrite32(0x2, framePtr + (0x0300/4)); 			//Enable Long capture
            iowrite32(0x1, framePtr + (0x2100/4)); 	                //Nchirps set to 1
            iowrite32(numSamples, framePtr + (0x800/4)); 	        //Nsamples
            //iowrite32(0x1, framePtr + (0x0400/4)); 	        //Trigger
        }
        else {
            pr_alert("[%s:%d] Incorrect Frame Triggering\n",__FILE__,__LINE__);
        }
    }
    else{
        pr_alert("[%s:%d] Copy From user Failed \n",__FILE__,__LINE__);
        return -1;
    }
    return PAGE_SIZE;
}

static ssize_t long_cap_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
    return snprintf(buf, PAGE_SIZE, "long_decimation_capture_show Not implemented\n");
}

static ssize_t long_cap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    int numSamples;
    int numChirps;
    numSamples = 0;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    if (count > 0) {
        ret = sscanf(buf, "%d %d", &numChirps, &numSamples);
        if (ret >=1){ //Check that number of samples is passed
            iowrite32(0x3, framePtr + (0x2D00/4)); 			//Reset Interrupts
            //iowrite32(0x01, framePtr + (0x0C00/4));
            iowrite32(0x2, framePtr + (0x0300/4)); 	        //Enable Long capture
            iowrite32(numSamples, framePtr + (0x800/4)); 	//Nsamples
            iowrite32(numChirps, framePtr + (0x2100/4)); 	//Nsamples
            //iowrite32(0x1, framePtr + (0x2100/4)); 	    //Nchirps set to 1
            //iowrite32(0x1, framePtr + (0x0400/4)); 		//Trigger
        }
        else {
            pr_alert("[%s:%d] Copy From user Failed \n",__FILE__,__LINE__);
        }
        return PAGE_SIZE;
    }
    else{
        pr_alert("[%s:%d] Copy From user Failed \n",__FILE__,__LINE__);
        return -1;
    }
}
#endif /*UNUSED_ATTR */
#ifdef DEBUG_APPS
/* Loop Back test */
static ssize_t loopback_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "loopback_show is Not implemented\n");
}

static ssize_t loopback_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    uint32_t Nsamples = 0, Ncycle = 0, Nchirps =0, setting = 0;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    
    if (count > 0) {
        ret = sscanf(buf, "%x %x %x %x", &setting, &Nsamples, &Ncycle, &Nchirps);
        if (ret >= 1){
            iowrite32( setting, framePtr + ( CSI2RX_SETTINGS/4));
            iowrite32( Nsamples, framePtr + ( CSI2RX_SAM_PER_FRAME/4));
            iowrite32( Ncycle, framePtr + ( CSI2RX_N_CYCLE/4));
            iowrite32( Nchirps, framePtr + ( CSI2RX_N_CHIRPS1/4));
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
/* Mixel configuration */
static ssize_t mixelconfig_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
    return snprintf(buf, PAGE_SIZE, "mixelconfig_show Not implemented\n");
}

static ssize_t mixelconfig_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    uint32_t rx_cfg_mode = 0, cfg_flush_cnt = 0, cfg_wtdg_cnt = 0, cfg_payld_0 = 0;
    uint32_t rx_clk_lane = 0, rx_lane0 = 0, rx_lane1 = 0;

#ifdef DEBUG
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
#endif /* DEBUG */
    if (count > 0) {
         ret = sscanf(buf, "%x %x %x %x %x %x %x", &rx_cfg_mode, &cfg_flush_cnt, &cfg_wtdg_cnt, &cfg_payld_0, &rx_clk_lane, &rx_lane0, &rx_lane1 );

        if (ret >= 1){ //Check that number of samples is passed

            iowrite32(rx_cfg_mode , MixelFramePtr + ( RX_CFG_MODE_OFFSET/4));
            iowrite32(cfg_flush_cnt , MixelFramePtr + ( CFG_FLUSH_COUNT_OFFSET/4));
            iowrite32(cfg_wtdg_cnt , MixelFramePtr + ( CFG_WATCHDOG_COUNT_OFFSET/4));
            iowrite32(cfg_payld_0 , MixelFramePtr + ( CFG_DISABLE_PAYLOAD_0_OFFSET/4)); /*DISABLE_DATA*/
            iowrite32(rx_clk_lane , MixelFramePtr + ( RX_CLOCK_LANE_OFFSET/4)); /* CFG_CLOCK */
            iowrite32(rx_lane0 , MixelFramePtr + ( RX_LANE_0_OFFSET/4)); /* CFG_LANE0 */
            iowrite32(rx_lane1 , MixelFramePtr + ( RX_LANE_1_OFFSET/4)); /* CFG_LANE1 */
        }
        else {
            pr_alert("[%s:%d] Copy From user Failed \n",__FILE__,__LINE__);
            return -1;
        }
        return PAGE_SIZE;
    }
    else{
        pr_alert("[%s:%d] Copy From user Failed \n",__FILE__,__LINE__);
        return -1;
    }
}
#endif /* DEBUG_APPS */
static ssize_t totalFrameNumberShow(struct device *dev, struct device_attribute *attr, char *buffer)
{
    #ifdef DEBUG
    pr_info("[%s:%d] After Capture totalFrameNumber =%lu\n",__FILE__,__LINE__, totalFrameNumber);
    #endif
    return sprintf(buffer,"%u\n",totalFrameNumber);
}
static ssize_t totalBytesWrittenShow(struct device *dev, struct device_attribute *attr, char *buffer)
{
    #ifdef DEBUG
    pr_info("[%s:%d] Written successful, totalBytes =%lu\n",__FILE__,__LINE__, totalBytesWritten);
    #endif
    return sprintf(buffer,"%u\n",totalBytesWritten);
}

static ssize_t radarCycleCountShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{

#ifdef DEBUG
    pr_info("[%s:%d] Read successful, radar Count =%lu\n",__FILE__,__LINE__, radar_cycle_count);
#endif
    return sprintf(resetBuf,"%u\n",radar_cycle_count);
}
static ssize_t radarCycleCountStore(struct device *dev, struct device_attribute *attr, const char *resetBuf, size_t count)
{
    tm_year = 0;
    tm_mday = 0;
    tm_mon  = 0;
    tm_hour = 0;
    tm_min  = 0;
    tm_sec  = 0;
    sscanf(resetBuf, "%u",&radar_cycle_count);
    radarCycConst = radar_cycle_count;
    if(radarCycConst == MAX_RADAR_CYCLE_COUNT)
    {
	    radarCycConst = 0x0; // Infinity Value Max Value Updated a 0 in the Filename 
    }
#ifdef DEBUG
    pr_info("[%s:%d] radar_cycle_count(%x) is stored successfully\r\n",__FILE__,__LINE__,radar_cycle_count);
#endif
    return PAGE_SIZE;
}

static ssize_t usbWriteCompleteShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{
#ifdef DEBUG
    pr_info("[%s:%d] Read successful, USB Write Complete =%d\n",__FILE__,__LINE__, usbWriteComplete);
#endif
#ifdef DEBUG_APPS
    return sprintf(resetBuf,"%d %s",usbWriteComplete,filename);
#else 
    return sprintf(resetBuf,"%d",usbWriteComplete);
#endif /*DEBUG_APPS*/
}

static ssize_t usbCaptureShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{
#ifdef DEBUG
    pr_info("[%s:%d] Read successful, USB Cap =%d\n",__FILE__,__LINE__, usbCapEn);
#endif
    return sprintf(resetBuf,"%d", usbCapEn);
}
static ssize_t usbCaptureStore(struct device *dev, struct device_attribute *attr, const char *resetBuf, size_t count)
{
    sscanf(resetBuf, "%d",&usbCapEn);
#ifdef DEBUG
    pr_info("[%s:%d] usbCapEn(%x) is stored successfully\r\n",__FILE__,__LINE__,usbCapEn);
#endif /* DEBUG */
    return PAGE_SIZE;
}


static ssize_t fpgaResetShow(struct device *dev, struct device_attribute *attr, char *resetBuf)
{
#ifdef DEBUG
    pr_info("[%s:%d] Read successful, FPGAresetValue =%d\n",__FILE__,__LINE__, resetDone);
#endif
    return sprintf(resetBuf,"%d", resetDone);
}
static ssize_t fpgaResetStore(struct device *dev, struct device_attribute *attr, const char *resetBuf, size_t count)
{
    totalFrameNumber = 0;
    sscanf(resetBuf, "%d",&resetDone);
    atomic_set(&counter_th, 0);
#ifdef TIME_STAMP_EN
    atomic_set(&counter_bh, 0);
    atomic_set(&counter_irq, 0);
#endif
#ifdef DEBUG
    pr_info("[%s:%d] resetDone(%x) is stored successfully\r\n",__FILE__,__LINE__,resetDone);
#endif
    return PAGE_SIZE;
}

static DEVICE_ATTR(totalFrameNumberRegister, S_IRUGO, totalFrameNumberShow, NULL);       //The Frame number is incremented after every capture of radar cycle.
static DEVICE_ATTR(totalBytesWrittenRegister, S_IRUGO, totalBytesWrittenShow, NULL);       //Number of bytes written into the disk
static DEVICE_ATTR(radarCycleCountRegister, S_IWUSR | S_IRUGO , radarCycleCountShow, radarCycleCountStore);       //The USB Write/Or Failure Status
static DEVICE_ATTR(usbWriteCompleteRegister, S_IRUGO, usbWriteCompleteShow, NULL);       //The USB Write/Or Failure Status
static DEVICE_ATTR(usbCaptureEnRegister, S_IWUSR | S_IRUGO, usbCaptureShow, usbCaptureStore);       //The capture has to be done to USB SSD
static DEVICE_ATTR(fpgaResetRegister, S_IWUSR | S_IRUGO, fpgaResetShow, fpgaResetStore);       //Write pid for process to be notified for interrupt
#ifdef DEBUG_APPS
static DEVICE_ATTR(mixel_config, S_IWUSR | S_IRUGO, mixelconfig_show, mixelconfig_store);	//Write pid for process to be notified for interrupt
static DEVICE_ATTR(loopback, S_IWUSR | S_IRUGO, loopback_show, loopback_store);
#endif /* DEBUG_APPS */
static DEVICE_ATTR(register_pid, S_IWUSR | S_IRUGO, pid_show, pid_store);	//Write pid for process to be notified for interrupt
#ifdef UNUSED_ATTR
static DEVICE_ATTR(capture, S_IWUSR | S_IRUGO, capture_show, capture_store);	//Trigger Capture and show results from capture
static DEVICE_ATTR(func_80_msps_cap, S_IWUSR | S_IRUGO, func_80_msps_cap_show, func_80_msps_cap_store);	//functional Capture and show results from long capture
static DEVICE_ATTR(long_80_msps_cap, S_IWUSR | S_IRUGO, long_80_msps_cap_show, long_80_msps_cap_store);	//long Capture 80msps and show results from long 80msps capture
static DEVICE_ATTR(long_cap, S_IWUSR | S_IRUGO, long_cap_show, long_cap_store);	//long decimation Capture and show results from long decimation capture
#endif /*UNUSED_ATTR*/
//================================= Device Constructors/Destructors ==================================================
static int csi2rx_construct_fpga_device(struct csi2rx_fpga_dev *dev, int minor, struct class *class)
{
    int err = 0;
    dev_t devno = MKDEV(csi2rx_dev_major, minor);
    struct device *device = NULL;
    BUG_ON(dev == NULL || class == NULL);

    mutex_init(&dev->csi2rx_mutex);

    cdev_init(&dev->cdev, &csi2rx_control_fops);
    dev->cdev.owner = THIS_MODULE;

    err = cdev_add(&dev->cdev, devno, 1);
    if (err)
    {
        pr_warn("[%s:%d] [target] Error %d while trying to add %s%d",__FILE__,__LINE__, err, "Custom PL control device %d", minor); 
        return err;
    }

    device = device_create(class, NULL, devno, NULL, "nxp_rx_control");

    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        pr_warn("[%s:%d] [target] Error %d while trying to create %s%d",__FILE__,__LINE__, err,  "Custom PL control device %d", minor-1);
        cdev_del(&dev->cdev);
        return err;
    }
    gdevice = device;
    /* device attributes on sysfs */
   
    err = device_create_file(device, &dev_attr_totalFrameNumberRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_totalBytesWrittenRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_radarCycleCountRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    } 
    err = device_create_file(device, &dev_attr_usbWriteCompleteRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_usbCaptureEnRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_fpgaResetRegister);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_register_pid);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
#ifdef UNUSED_ATTR
    err = device_create_file(device, &dev_attr_capture);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_func_80_msps_cap);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute functional_80msps\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_long_80_msps_cap);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute long_capture_80msps\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_long_cap);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute long_capture_decimation\n",__FILE__,__LINE__);
        return err;
    }
#endif /*UNUSED_ATTR*/
    #ifdef DEBUG_APPS
	err = device_create_file(device, &dev_attr_loopback);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    err = device_create_file(device, &dev_attr_mixel_config);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
        return err;
    }
    #endif /*DEBUG_APPS*/

    return 0;
}

static int csi2rx_construct_adc_device(struct csi2rx_adc_dev *dev, int minor, struct class *class)
{
    int err = 0;
    dev_t devno = MKDEV(csi2rx_dev_major, minor);
    struct device *device = NULL;

    BUG_ON(dev == NULL || class == NULL);

    if((minor - NUM_UUTS -1)%NUM_ADC_CHANNELS == 0){	//The frame struct is shared by every NUM_ADC_CHANNELS
        csi2rx_frame_devices[MY_ADC_FRAME(minor)].frame_data = NULL;     /* Memory is to be allocated when the device is opened the first time */
        csi2rx_frame_devices[MY_ADC_FRAME(minor)].frame_size = 0;
    }

    dev->my_frame = &csi2rx_frame_devices[MY_ADC_FRAME(minor)];	//Assign the correspondent shared structure
    dev->adc_code = (minor - NUM_UUTS -1)%NUM_ADC_CHANNELS; 	//Store channel number relative to uut

    mutex_init(&dev->csi2rx_mutex);

    cdev_init(&dev->cdev, &csi2rx_adc_fops);
    dev->cdev.owner = THIS_MODULE;

    err = cdev_add(&dev->cdev, devno, 1);
    if (err)
    {
        pr_warn("[%s:%d] [target] Error %d while trying to add %s%d",__FILE__,__LINE__, err, "adc%d", (minor-NUM_UUTS)); 
        return err;
    }

    device = device_create(class, NULL, devno, NULL, "adc%d", (minor-NUM_UUTS));

    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        pr_warn("[%s:%d] [target] Error %d while trying to create %s%d",__FILE__,__LINE__, err, "adc%d", (minor-NUM_UUTS));
        cdev_del(&dev->cdev);
        return err;
    }

    return 0;
}

static void csi2rx_destroy_fpga_device(struct csi2rx_fpga_dev *dev, int minor, struct class *class)
{
    BUG_ON(dev == NULL || class == NULL);
    device_destroy(class, MKDEV(csi2rx_dev_major, minor));
    cdev_del(&dev->cdev);
    mutex_destroy(&dev->csi2rx_mutex);
    return;
}

static void csi2rx_destroy_adc_device(struct csi2rx_adc_dev *dev, int minor,struct class *class)
{
    BUG_ON(dev == NULL || class == NULL);
    cdev_del(&dev->cdev);
    device_destroy(class, MKDEV(csi2rx_dev_major, minor));
    mutex_destroy(&dev->csi2rx_mutex);
    return;
}
static void csi2rx_cleanup_module(int devices_to_destroy)
{
    int i;

    /* Get rid of character devices (if any exist) */
    if (devices_to_destroy == 0) goto out;
    csi2rx_destroy_fpga_device(&csi2rx_control_dev, 0, csi2rx_class);

    if (devices_to_destroy > 1){
        for(i = NUM_UUTS + 1  ; i<min(NUM_DEVICES, devices_to_destroy); i++){
            csi2rx_destroy_adc_device(&csi2rx_adc_devices[i-(NUM_UUTS +1  )], i, csi2rx_class);
        }
        kfree(csi2rx_frame_devices);
        kfree(csi2rx_adc_devices);
    }
out:
    if (csi2rx_class)
    {
        class_destroy(csi2rx_class);
    }
    unregister_chrdev_region(device_number, NUM_DEVICES);
    return;
}

//**************************** Module Init and Release ***************************************
static int __init nxp_csi2rx_driver_init(void)
{
    int err = 0;
    struct device_node *np = NULL;
    int devices_to_destroy = 0;
    int i = 0;
    uint32_t MixelOrXlnx = 0;
    //detectedChirps = 0;

    csi2rx_usb_dev = (struct nxp_datacap_usb_rd_wr_local *) kmalloc(sizeof(struct nxp_datacap_usb_rd_wr_local), GFP_KERNEL);
    if (!csi2rx_usb_dev) {
                pr_warn("Cound not allocate nxp-datacap-usb-rd-wr device\n");
                return -ENOMEM;
    }

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    /* Get a range of minor numbers (starting with 0) to work with */
    err = alloc_chrdev_region(&device_number, 0, NUM_DEVICES, CSI2RX_MODNAME);
    if (err < 0) {
        pr_warn("[%s:%d] [target] alloc_chrdev_region() failed\n",__FILE__,__LINE__);
	kfree(csi2rx_usb_dev);
        return err;
    }
    csi2rx_dev_major = MAJOR(device_number);

    /* Create device class (before allocation of the array of devices) */
    csi2rx_class = class_create(THIS_MODULE, CSI2RX_MODNAME);
    if (IS_ERR(csi2rx_class)) {
        err = PTR_ERR(csi2rx_class);
        goto fail;
    }

    /* Allocate the array of frame devices, there are only NUM_UUTS frame devices even though there are NUM_UUTS * NUM_ADC_CHANNELS devices */
    csi2rx_frame_devices = (struct csi2rx_frame_dev *)kzalloc( NUM_UUTS * sizeof(struct csi2rx_frame_dev), GFP_KERNEL);
    if (csi2rx_frame_devices == NULL) {
        err = -ENOMEM;
        goto fail;
    }

    /* Allocate the array of frame devices, there are only NUM_UUTS frame devices even though there are NUM_UUTS * NUM_ADC_CHANNELS devices */
    csi2rx_adc_devices = (struct csi2rx_adc_dev *)kzalloc( NUM_UUTS * NUM_ADC_CHANNELS * sizeof(struct csi2rx_adc_dev), GFP_KERNEL);
    if (csi2rx_adc_devices == NULL) {
        err = -ENOMEM;
        goto fail;
    }

    /* Construct devices: 0 is generic controller, 1 to NUM_UUTS is spi_devices, NUM_UUTS + 1 to NUM_UUTS + 16 is adc channels*/
    err = csi2rx_construct_fpga_device(&csi2rx_control_dev, 0, csi2rx_class);
    if (err) {
        devices_to_destroy = 1;
        goto fail;
    }
    csi2rx_usb_dev->dev_ptr = gdevice;
    dev_set_drvdata(csi2rx_usb_dev->dev_ptr, csi2rx_usb_dev);
    for (i = NUM_UUTS + 1; i < NUM_DEVICES; i++){
        err = csi2rx_construct_adc_device(&csi2rx_adc_devices[i-(NUM_UUTS + 1)], i, csi2rx_class);
        if (err) {
            devices_to_destroy = i + 1;
            goto fail;
        }
    }

    //Map AXI addresses! 
    framePtr =  ioremap(CSI2_RX_START_ADDR, (CSI2_RX_END_ADDR - CSI2_RX_START_ADDR) + 4 );
    if(framePtr == NULL)
    {
        pr_alert("[%s:%d] Frame ptr is NULL and ioremap is failed .....\n",__FILE__,__LINE__);
        goto r_map;
    }

    MixelOrXlnx = ioread32(framePtr + (CSI2RX_VERSION/4));    //Get the version info
    if((MixelOrXlnx & VERSION_BIT_MASKING) == MIXEL_VERSION)
    {
        mixelEnable = true;
        pr_info("[%s:%d] Mixel CSI-RX IP is enabled ......\n",__FILE__,__LINE__);
        //Map MIXEL addresses! 
        MixelFramePtr =  ioremap(CSI2_RX_START_PHY_ADDR, (CSI2_RX_END_PHY_ADDR - CSI2_RX_START_PHY_ADDR) + 4 );
        if(MixelFramePtr == NULL)
        {
            pr_alert("[%s:%d] Frame ptr is NULL and ioremap is failed .....\n",__FILE__,__LINE__);
            goto r_map;
        }
        else
        {
            iowrite32(DEFAULT_MIXEL_RX_CFG_MODE_CONFIG, MixelFramePtr + ( RX_CFG_MODE_OFFSET/4));
            iowrite32(DEFAULT_MIXEL_RX_IRQ_ENABLE_CONFIG, MixelFramePtr + ( RX_IRQ_ENABLE_OFFSET/4));
            iowrite32(DEFAULT_MIXEL_RX_CLOCK_LANE_CONFIG, MixelFramePtr + ( RX_CLOCK_LANE_OFFSET/4)); /* CFG_CLOCK */
            iowrite32(DEFAULT_MIXEL_RX_LANE_0_CONFIG, MixelFramePtr + ( RX_LANE_0_OFFSET/4)); /* CFG_LANE0 */
            iowrite32(DEFAULT_MIXEL_RX_LANE_1_CONFIG, MixelFramePtr + ( RX_LANE_1_OFFSET/4)); /* CFG_LANE1 */
        }
    }

    //Obtain Interrupt Numbers
    np = of_find_node_by_name(NULL,"nxp-csi2rx-driver"); // the <node_name> is the one before “@” sign in dtsi file.
    FRAME_INTERRUPT_RX = irq_of_parse_and_map(np,0);

    //Register Interrupt for Frame Grabber (frame ready)
    err = request_irq(FRAME_INTERRUPT_RX, data_isr, 0, CSI2RX_MODNAME ".frameIsrRx", NULL);
    if (err < 0) {
        pr_alert("[%s:%d] request_irq %d for module %s failed with %d\n",__FILE__,__LINE__, FRAME_INTERRUPT_RX, CSI2RX_MODNAME ".frameIsrRx", err);
        goto r_irq;
    }
    else { pr_info("[%s:%d] #>> Frame Interrupt was succesfully registered: %d\n",__FILE__,__LINE__, FRAME_INTERRUPT_RX);}

    INIT_WORK(&csi2_work, workq_fun);
    /* task is initialized to NULL */
    t = NULL;
#ifdef TIME_STAMP_EN
    atomic_set(&counter_bh, 0);
    atomic_set(&counter_irq, 0);
#endif
    atomic_set(&counter_th, 0);
    return 0; /* success */

r_irq:
    free_irq(FRAME_INTERRUPT_RX, NULL);
r_map:
    iounmap(framePtr);
    if(mixelEnable)
    iounmap(MixelFramePtr);
fail:
    csi2rx_cleanup_module(devices_to_destroy);
    if(csi2rx_usb_dev)
    	kfree(csi2rx_usb_dev);

    return err;
}

static void __exit nxp_csi2rx_driver_exit(void)
{
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    iounmap(framePtr);
    if(mixelEnable)
    iounmap(MixelFramePtr);
    free_irq(FRAME_INTERRUPT_RX, NULL);

    csi2rx_cleanup_module(NUM_DEVICES);
    if(csi2rx_usb_dev)
    	kfree(csi2rx_usb_dev);
     if(g_cfile != NULL)
     {
        pr_info("Freeing the path of SSD and closing the Device(SSD) File \n");
	if (file_count(g_cfile)) {
     		filp_close(g_cfile, NULL);
		g_cfile = NULL;
	}
     }
     if(path != NULL)
     {
     	kfree(path);
     }

}

module_init(nxp_csi2rx_driver_init);
module_exit(nxp_csi2rx_driver_exit);

//********************************** Licensing ***************************************************
MODULE_DESCRIPTION("NXP driver to control the custom PL block for csi2rx devices communication and data capture");
MODULE_LICENSE("GPL v2");


