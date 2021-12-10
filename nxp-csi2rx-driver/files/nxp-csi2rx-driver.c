/*
* Copyright 2021 NXP
*
* SPDX-License-Identifier: GPL-2.0
* The GPL-2.0 license for this file can be found in the COPYING.GPL file
* included with this distribution or at http://www.gnu.org/licenses/gpl-2.0.html
*/

#include "nxp-csi2rx-driver.h"

#define NUM_UUTS  1								//Max number of supported UUTs
#define NUM_ADC_CHANNELS 4						//Number of ADC channels per UUT
#define NUM_DEVICES (1 + NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS))	//Each uut will have a adc device and a frame device (do we need a status device? for interrupts and so on ...)
#define BUF_LEN 5000 							// Max length of the message from the device
#define SUCCESS 0								//Success code

#define MY_ADC_FRAME(I) ((I - NUM_UUTS -1)/NUM_ADC_CHANNELS)		//Macro to find out the corresponding frame device from adc device minor 


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
static unsigned int detectedChirps;
static struct task_struct *t;
static struct kernel_siginfo info;
 
//====================== Custom PL Control Device Functions ========================
static int device_open(struct inode *inode, struct file *filp){
    unsigned int mj = imajor(inode);
    unsigned int mn = iminor(inode);
    int i;

    struct csi2rx_fpga_dev *dev = &csi2rx_control_dev;

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
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
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);

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
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
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

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);

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
        printk(KERN_EMERG"Inside %s frameSizeBytes = %d count =%d\n ",__FUNCTION__,frameSizeBytes,count); 
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
    uint32_t irqRet = 0;

    pr_info("[%s:%d]  calling func %s \n",__FILE__,__LINE__, __func__);

    captureBuffer = ioread32(framePtr + (0x2B00/4));    //Get the buffer address
    EnableLonCap  = ioread32(framePtr + (0x0300/4)); 
    Enable_80msps = ioread32(framePtr + (0x0C00/4));

    //Workaround 
    if(Enable_80msps & 0xDEADBEEF)
    {
        pr_info("[%s:%d] Resetting Enable_80msps value to zero \n",__FILE__,__LINE__);
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

    frameSize =  ioread32(framePtr + (0x1000/4));	//Get value from FPGA register;

    irqRet = ioread32(framePtr + (0x2C00/4));

    if (irqRet != 0x2) {
        pr_alert("[%s:%d] Error During Frame Capture. IRQ: %d!!!\n",__FILE__,__LINE__, irqRet);
        goto out;
    }

    if(((EnableLonCap & 0x00000002) >> 1) == 1) {
        for (i=0; i<NUM_UUTS;i++) {
            csi2rx_frame_devices[i].frame_address = 0x40000000 + (0x10000000 * i);
            csi2rx_frame_devices[i].BypassOrDecimated = ((Enable_80msps & 0x00000008) >> 3);
            csi2rx_frame_devices[i].frame_size = frameSize;
        }
    }
    else {
        for (i=0; i<NUM_UUTS;i++) {
            csi2rx_frame_devices[i].frame_address = 0x40000000 + (0x10000000 * i) + frameOffset;
            csi2rx_frame_devices[i].BypassOrDecimated = ((Enable_80msps & 0x00000008) >> 3);
            csi2rx_frame_devices[i].frame_size = frameSize;
        }
    }

    /* send the signal */
    info.si_int = 0;     //real time signals may have 32 bits of data.
    ret = send_sig_info(SIGUSR1, &info, t);    //send the signal
    if (ret < 0) {
        printk("error sending signal\n");
        return IRQ_HANDLED;
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

static long nxp_csi2rx_control_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    uint32_t ret_val,offset,flength;
    struct csi2_rx_control_reg csi2_rx;
    
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
        default:
            pr_info("[%s:%d] Unknown IOCTL ...\n",__FILE__,__LINE__);
            break;
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
    return sprintf(buf,"Read successful, PID Value =%d\n", pid);
}

static ssize_t pid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d",&pid);
    printk("PID(%x) is stored successfully\r\n",pid);

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

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
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

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
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

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
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

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
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

/* Loop Back test */
static ssize_t loopback_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "loopback_show is Not implemented\n");
}

static ssize_t loopback_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    uint32_t Nsamples = 0, Ncycle = 0, Nchirps =0, setting = 0;

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    
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

static DEVICE_ATTR(loopback, S_IWUSR | S_IRUGO, loopback_show, loopback_store);

static DEVICE_ATTR(register_pid, S_IWUSR | S_IRUGO, pid_show, pid_store);	//Write pid for process to be notified for interrupt
static DEVICE_ATTR(capture, S_IWUSR | S_IRUGO, capture_show, capture_store);	//Trigger Capture and show results from capture
static DEVICE_ATTR(func_80_msps_cap, S_IWUSR | S_IRUGO, func_80_msps_cap_show, func_80_msps_cap_store);	//functional Capture and show results from long capture
static DEVICE_ATTR(long_80_msps_cap, S_IWUSR | S_IRUGO, long_80_msps_cap_show, long_80_msps_cap_store);	//long Capture 80msps and show results from long 80msps capture
static DEVICE_ATTR(long_cap, S_IWUSR | S_IRUGO, long_cap_show, long_cap_store);	//long decimation Capture and show results from long decimation capture
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

    /* device attributes on sysfs */
    err = device_create_file(device, &dev_attr_register_pid);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
    }
    err = device_create_file(device, &dev_attr_capture);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
    }
    err = device_create_file(device, &dev_attr_func_80_msps_cap);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute functional_80msps\n",__FILE__,__LINE__);
    }
    err = device_create_file(device, &dev_attr_long_80_msps_cap);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute long_capture_80msps\n",__FILE__,__LINE__);
    }
    err = device_create_file(device, &dev_attr_long_cap);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute long_capture_decimation\n",__FILE__,__LINE__);
    }
    err = device_create_file(device, &dev_attr_loopback);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
    }

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
    struct csi2rx_frame_dev *frame_dev;

    BUG_ON(dev == NULL || class == NULL);
    frame_dev = dev->my_frame;
    if(frame_dev->frame_data !=NULL) 
        kfree(frame_dev->frame_data);
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
    //detectedChirps = 0;

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    /* Get a range of minor numbers (starting with 0) to work with */
    err = alloc_chrdev_region(&device_number, 0, NUM_DEVICES, KBUILD_MODNAME);
    if (err < 0) {
        pr_warn("[%s:%d] [target] alloc_chrdev_region() failed\n",__FILE__,__LINE__);
        return err;
    }
    csi2rx_dev_major = MAJOR(device_number);

    /* Create device class (before allocation of the array of devices) */
    csi2rx_class = class_create(THIS_MODULE, KBUILD_MODNAME);
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
    for (i = NUM_UUTS + 1; i < NUM_DEVICES; i++){
        err = csi2rx_construct_adc_device(&csi2rx_adc_devices[i-(NUM_UUTS + 1)], i, csi2rx_class);
        if (err) {
            devices_to_destroy = i + 1;
            goto fail;
        }
    }

    //Obtain Interrupt Numbers
    np = of_find_node_by_name(NULL,"nxp-csi2rx-driver"); // the <node_name> is the one before “@” sign in dtsi file.
    FRAME_INTERRUPT_RX = irq_of_parse_and_map(np,0);

    //Register Interrupt for Frame Grabber (frame ready)
    err = request_irq(FRAME_INTERRUPT_RX, data_isr, 0, KBUILD_MODNAME ".frameIsrRx", NULL);
    if (err < 0) {
        pr_alert("[%s:%d] request_irq %d for module %s failed with %d\n",__FILE__,__LINE__, FRAME_INTERRUPT_RX, KBUILD_MODNAME ".frameIsrRx", err);
        goto r_irq;
    }
    else { pr_info("[%s:%d] Frame Interrupt was succesfully registered: %d\n",__FILE__,__LINE__, FRAME_INTERRUPT_RX);}

    //Map AXI addresses! 
    framePtr =  ioremap(CSI2_RX_START_ADDR, (CSI2_RX_END_ADDR - CSI2_RX_START_ADDR) + 4 );
    if(framePtr == NULL)
    {
        pr_alert("[%s:%d] Frame ptr is NULL and ioremap is failed .....\n",__FILE__,__LINE__);
        goto r_map;
    }


    return 0; /* success */

r_irq:
    free_irq(FRAME_INTERRUPT_RX, NULL);
r_map:
    iounmap(framePtr);
fail:
    csi2rx_cleanup_module(devices_to_destroy);
    return err;
}

static void __exit nxp_csi2rx_driver_exit(void)
{
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    iounmap(framePtr);
    free_irq(FRAME_INTERRUPT_RX, NULL);

    csi2rx_cleanup_module(NUM_DEVICES);
}

module_init(nxp_csi2rx_driver_init);
module_exit(nxp_csi2rx_driver_exit);

//********************************** Licensing ***************************************************
MODULE_DESCRIPTION("NXP driver to control the custom PL block for csi2rx devices communication and data capture");
MODULE_LICENSE("GPL v2");
