/*
* Copyright 2021 NXP
*
* SPDX-License-Identifier: GPL-2.0
* The GPL-2.0 license for this file can be found in the COPYING.GPL file
* included with this distribution or at http://www.gnu.org/licenses/gpl-2.0.html
*/

#include "nxp-csi2tx-driver.h"

#define NUM_UUTS  1								//Max number of supported UUTs
#define NUM_ADC_CHANNELS 4						//Number of ADC channels per UUT
#define NUM_DEVICES (1 + NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS))	//Each uut will have a adc device and a frame device (do we need a status device? for interrupts and so on ...)
#define SUCCESS 0								//Success code

#define MY_ADC_FRAME(I) ((I - NUM_UUTS -1)/NUM_ADC_CHANNELS)		//Macro to find out the corresponding frame device from adc device minor 

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
//================== Device Variables ========================
static dev_t csi2tx_dev_major , device_number;									//Device identifier
static struct csi2tx_frame_dev *csi2tx_frame_devices = NULL;		//Collection of Frame devices used by ADC devices
static struct csi2tx_adc_dev *csi2tx_adc_devices = NULL;			//Collection of ADC devices
static struct csi2tx_fpga_dev csi2tx_control_dev;					//Single instance of custom PL controller
static struct class *csi2tx_class = NULL;						//Driver class
//******************* Interrupt codes *********************
static int FRAME_INTERRUPT_TX = 0;					// Place to store OS interrupt code for SPI HW int	
//=================== Kernel operation variables ===================
static unsigned int *framePtr;
//static unsigned int detectedChirps;
static struct task_struct *t;
static struct kernel_siginfo info;

#define LOOP_BACK_TEST
#define CSI2_TX_SUBSYSTEM_START_ADDR    0xA0002000
#define CSI2_TX_SUBSYSTEM_END_ADDR      0xA0004000
#ifdef LOOP_BACK_TEST
static unsigned int *lbPtr = NULL; // Loop back 
#endif

//====================== Custom PL Control Device Functions ========================
static int device_open(struct inode *inode, struct file *filp){
    unsigned int mj = imajor(inode);
    unsigned int mn = iminor(inode);
    int i;

    struct csi2tx_fpga_dev *dev = &csi2tx_control_dev;

    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
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
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    module_put(THIS_MODULE);
    return SUCCESS;
}

static ssize_t device_read(struct file *filp, char __user * buffer, size_t length, loff_t * offset){
    int bytes_read;
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    return bytes_read;
}

static ssize_t device_write(struct file *filp, const char __user * buffer, size_t length, loff_t * offset){
    struct csi2tx_fpga_dev *dev;
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    dev = (struct csi2tx_fpga_dev *)filp->private_data;

    return length;
}


//============================== FRAME Device Functions ==============================
static int adc_open(struct inode *inode, struct file *filp){
    unsigned int mj = imajor(inode);
    unsigned int mn = iminor(inode);

    struct csi2tx_adc_dev *dev = NULL;
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);

    if (mj != csi2tx_dev_major || mn <= NUM_UUTS || mn > (NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS)))
    {
        pr_warn("[%s:%d] [target] " "No device found with minor=%d and major=%d\n",__FILE__,__LINE__, mj, mn);
        return -ENODEV; /* No such device */
    }

    /* store a pointer to struct cfake_dev here for other methods */
    dev = &csi2tx_adc_devices[mn - (NUM_UUTS + 1)];
    filp->private_data = dev; 

    if (inode->i_cdev != &dev->cdev)
    {
        pr_warn("[%s:%d] [target] open: internal error\n",__FILE__,__LINE__);
        return -ENODEV; /* No such device */
    }

    return 0;
}

static int adc_release(struct inode *inode, struct file *filp){
     iowrite32(0xFF , framePtr + (CSI2TX_IRQ_CLR/4));
     pr_info("[%s:%d] frame complete interrupt is cleared \n",__FILE__,__LINE__);
    return 0;
}

static ssize_t adc_write(struct file *filp, const char __user * buffer, size_t length, loff_t * offset){
    uint8_t addr_ptr = 0;
    uint8_t frame_ptr = 0;
    struct csi2tx_adc_dev *dev;
    struct csi2tx_frame_dev *frame_dev;
    ssize_t retval;
    dev = (struct csi2tx_adc_dev *)filp->private_data;
    frame_dev = (struct csi2tx_frame_dev *)dev->my_frame;
    retval = 0;
    
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    
    frame_ptr = ioread32(framePtr + (CSI2TX_FRAME_PTR/4)); 
    frame_dev->frame_address = ioread32(framePtr + (0x00C0 +frame_ptr * 4)/4);
    frame_dev->frame_size = length;
    
    frame_dev->frame_data  = (uint64_t *)ioremap(frame_dev->frame_address, 0x4000000  ); 
    if(frame_dev->frame_data == NULL)
    {
        pr_info("[%s:%d] frame_dev->frame_data buffer is null \n",__FILE__,__LINE__);
        return -1;
    }

	copy_from_user(frame_dev->frame_data, buffer, length);
    //pr_info("[%s:%d] Buffer length %d \n",__FILE__,__LINE__,length);
   
    iounmap(frame_dev->frame_address);
    mutex_unlock(&dev->csi2tx_mutex);

	return length;
}

//************************************* HW IO ISRs ****************************************
static irqreturn_t data_isr(int irq,void*dev_id) {      
    int i = 0;
    uint8_t addr_ptr = 0;
    uint32_t irqRet = 0;
    int ret;

    pr_info("[%s:%d]  calling func %s \n",__FILE__,__LINE__, __func__);
    
    irqRet = ioread32(framePtr + (0x0070/4));
    if ((irqRet & 0x2) != 0x2) {
        pr_alert("[%s:%d] Error During Frame transmitted. IRQ: %X!!!\n",__FILE__,__LINE__, irqRet);
        goto out;
    }
    
    info.si_int = 1;          //real time signals may have 32 bits of data.
    ret = send_sig_info(SIGUSR1, &info, t);    //send the signal
    if (ret < 0) {
        printk("error sending signal\n");
        return IRQ_HANDLED;
    }

out:
    
    return IRQ_HANDLED;
}
//************************************ IOCTL *****************************************
void csi2tx_reg_write( struct csi2_tx_control_reg csi2_tx)
{
    if(csi2_tx.status)
    {
        iowrite32( csi2_tx.vc0_lines, lbPtr + (0x40/4));
    }
    iowrite32( csi2_tx.nframesTrans, framePtr + (CSI2TX_NFRAMES_TRNSMIT/4));
}

void ioctl_csi2tx_irq_reset( void )
{
        iowrite32( 0xFF, framePtr + (CSI2TX_IRQ_CLR/4));
}

static long nxp_csi2tx_control_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    uint32_t ret_val,offset;
    struct csi2_tx_control_reg csi2_tx;

 switch(cmd) {
        case  IOCTL_REG_WRITE_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            csi2tx_reg_write(csi2_tx );
            copy_to_user((struct csi2_tx_control_reg*)arg, &csi2_tx,sizeof(struct csi2_tx_control_reg));
            break;
        case IOCTL_FRAME_START_TX:
            iowrite32( 0x1, framePtr + (CSI2TX_FRAME_START/4));
            break;
        case IOCTL_FRAME_LEN_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            iowrite32( csi2_tx.frameLen, framePtr + (CSI2TX_FRAME_LEN/4));
            iowrite32( csi2_tx.line_Len, framePtr + (CSI2TX_LINE_LEN/4));
            iowrite32( csi2_tx.vert_Lines, framePtr + (CSI2TX_VERT_LINE/4));
            break;
        case IOCTL_IRQ_READ_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            csi2_tx.irqStat = ioread32(framePtr +(CSI2TX_IRQ_STAT/4));
            csi2_tx.framesCnt = ioread32(framePtr +(CSI2TX_FRAMES_TRNSMIT_CNT/4));
            copy_to_user((struct csi2_tx_control_reg*)arg, &csi2_tx, sizeof( struct csi2_tx_control_reg ));
            break;
        case IOCTL_IRQ_RESET_TX :
            ioctl_csi2tx_irq_reset();
            break;
        case IOCTL_BUILD_INFO_READ_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            csi2_tx.buildID = ioread32(framePtr +(CSI2TX_BUILD_ID/4));
            csi2_tx.buildDate = ioread32(framePtr +(CSI2TX_BUILD_DATE/4));
            csi2_tx.buildTime = ioread32(framePtr +(CSI2TX_BUILD_TIME/4));
            csi2_tx.status = true;
            copy_to_user((struct csi2_tx_control_reg*)arg, &csi2_tx, sizeof( struct csi2_tx_control_reg ));
            break;
        case IOCTL_DBG1_DBG2_READ_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            csi2_tx.calSum_dbg1 = ioread32(framePtr +(CSI2TX_CAL_SUM_DBG1/4));
            csi2_tx.calSum_dbg2 = ioread32(framePtr +(CSI2TX_CAL_SUM_DBG2/4));
            copy_to_user((struct csi2_tx_control_reg*)arg, &csi2_tx, sizeof( struct csi2_tx_control_reg ));
            break;
        case IOCTL_CONF_SETTINGS_TX:
            copy_from_user(&csi2_tx, (struct csi2_tx_control_reg *)arg, sizeof(struct csi2_tx_control_reg));
            iowrite32( csi2_tx.setting, framePtr + (CSI2TX_SETTINGS/4));
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
    return sprintf(buf,"[%s:%d] Read successful, PID Value =%d\n",__FILE__,__LINE__, pid);
}

static ssize_t pid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d",&pid);
    pr_info("[%s:%d] PID(%x) is stored successfully\r\n",__FILE__,__LINE__,pid);

    rcu_read_lock();
    t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);
    if(t == NULL){
        pr_info(" [%s:%d] no such pid\n",__FILE__,__LINE__);
        rcu_read_unlock();
        return -1;
    }
    rcu_read_unlock();
    
    memset(&info, 0, sizeof(struct kernel_siginfo));
    info.si_signo = SIGUSR1;
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
    uint32_t vc0_lines = 0, line_len = 0;

    if (count > 0) {
        ret = sscanf(buf, "%x", &line_len);
        if (ret >=1){ 
            iowrite32( 0x1, lbPtr + (0x00/4));
            //iowrite32( vc0_lines, lbPtr + (0x40/4));
            iowrite32( line_len, lbPtr + (0x40/4));
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
    }
    err = device_create_file(device, &dev_attr_loopback);
    if (err < 0) {
        pr_err("[%s:%d] Cant create device attribute\n",__FILE__,__LINE__);
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
        csi2tx_frame_devices[MY_ADC_FRAME(minor)].frame_data = NULL;     /* Memory is to be allocated when the device is opened the first time */
        csi2tx_frame_devices[MY_ADC_FRAME(minor)].frame_size = 0;
    }

    dev->my_frame = &csi2tx_frame_devices[MY_ADC_FRAME(minor)];	//Assign the correspondent shared structure
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
    struct csi2tx_frame_dev *frame_dev;

    BUG_ON(dev == NULL || class == NULL);
    frame_dev = dev->my_frame;
    if(frame_dev->frame_data !=NULL) 
        kfree(frame_dev->frame_data);
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
    csi2tx_destroy_fpga_device(&csi2tx_control_dev, 0, csi2tx_class);

    if (devices_to_destroy > 1){
        for(i = NUM_UUTS + 1  ; i<min(NUM_DEVICES, devices_to_destroy); i++){
            csi2tx_destroy_adc_device(&csi2tx_adc_devices[i-(NUM_UUTS +1  )], i, csi2tx_class);
        }
        kfree(csi2tx_frame_devices);
        kfree(csi2tx_adc_devices);
    }
out:
    if (csi2tx_class)
    {
        class_destroy(csi2tx_class);
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

    /* Get a range of minor numbers (starting with 0) to work with */
    err = alloc_chrdev_region(&device_number, 0, NUM_DEVICES, KBUILD_MODNAME);
    if (err < 0) {
        pr_warn("[%s:%d] [target] alloc_chrdev_region() failed\n",__FILE__,__LINE__);
        return err;
    }
    csi2tx_dev_major = MAJOR(device_number);

    /* Create device class (before allocation of the array of devices) */
    csi2tx_class = class_create(THIS_MODULE, KBUILD_MODNAME);
    if (IS_ERR(csi2tx_class)) {
        err = PTR_ERR(csi2tx_class);
        goto fail;
    }

    /* Allocate the array of frame devices, there are only NUM_UUTS frame devices even though there are NUM_UUTS * NUM_ADC_CHANNELS devices */
    csi2tx_frame_devices = (struct csi2tx_frame_dev *)kzalloc( NUM_UUTS * sizeof(struct csi2tx_frame_dev), GFP_KERNEL);
    if (csi2tx_frame_devices == NULL) {
        err = -ENOMEM;
        goto fail;
    }

    /* Allocate the array of frame devices, there are only NUM_UUTS frame devices even though there are NUM_UUTS * NUM_ADC_CHANNELS devices */
    csi2tx_adc_devices = (struct csi2tx_adc_dev *)kzalloc( NUM_UUTS * NUM_ADC_CHANNELS * sizeof(struct csi2tx_adc_dev), GFP_KERNEL);
    if (csi2tx_adc_devices == NULL) {
        err = -ENOMEM;
        goto fail;
    }

    /* Construct devices: 0 is generic controller, 1 to NUM_UUTS is  NUM_UUTS + 1 to NUM_UUTS + 4  is adc channels*/
    err = csi2tx_construct_fpga_device(&csi2tx_control_dev, 0, csi2tx_class);
    if (err) {
        devices_to_destroy = 1;
        goto fail;
    }
    for (i = NUM_UUTS + 1; i < NUM_DEVICES; i++){
        err = csi2tx_construct_adc_device(&csi2tx_adc_devices[i-(NUM_UUTS + 1)], i, csi2tx_class);
        if (err) {
            devices_to_destroy = i + 1;
            goto fail;
        }
    }

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

    //Map AXI addresses! 
    framePtr =  ioremap(CSI2_TX_START_ADDR, (CSI2_TX_END_ADDR - CSI2_TX_START_ADDR) + 4 );
    if(framePtr == NULL)
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

    return 0; /* success */

r_irq:
    free_irq(FRAME_INTERRUPT_TX, NULL);
r_map:
    iounmap(framePtr);
#ifdef LOOP_BACK_TEST
    iounmap(lbPtr);
#endif
fail:
    csi2tx_cleanup_module(devices_to_destroy);
    return err;
}

static void __exit nxp_csi2tx_driver_exit(void)
{
    pr_info("[%s:%d] calling func %s \n",__FILE__,__LINE__,__func__);
    iounmap(framePtr);
#ifdef LOOP_BACK_TEST
    iounmap(lbPtr);
#endif
    free_irq(FRAME_INTERRUPT_TX, NULL);

    csi2tx_cleanup_module(NUM_DEVICES);
}

module_init(nxp_csi2tx_driver_init);
module_exit(nxp_csi2tx_driver_exit);

//********************************** Licensing ***************************************************
MODULE_DESCRIPTION("NXP driver to control the custom PL block for csi2tx devices communication and data capture");
MODULE_LICENSE("GPL v2");
