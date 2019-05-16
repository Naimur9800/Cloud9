#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
//#include <asm/atomic.h>
#include <linux/slab.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h" 
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "ov8856mipiraw_Sensor.h"

#define PFX "ov8856_sunwin_otp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

static kal_uint16 ov8856_read_i2c(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, 0x20);

	return get_byte;
}

static void ov8856_write_i2c(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, 0x20);
}

struct otp_struct
{
	int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int lenc[240];
	int checksum;
}otp_info;

u16 otp_ov8856_af_data[2];
static int RG_Ratio_Typical= 0x14e;
static int BG_Ratio_Typical= 0x189;////modify by v3720 SWCR5093730A-VA OTP Report 17.4.27

int ov8856_read_otp(void)
{
	int otp_flag, addr, temp, i;

	ov8856_write_i2c(0x0100,0x01);
	mdelay(10);
	int temp1;
	temp1 = ov8856_read_i2c(0x5001);    

	ov8856_write_i2c(0x5001, (0x00 & 0x08) | (temp1 & (~0x08)));
	// read OTP into buffer
	ov8856_write_i2c(0x3d84, 0xC0);
	ov8856_write_i2c(0x3d88, 0x70); // OTP start address
	ov8856_write_i2c(0x3d89, 0x10);
	ov8856_write_i2c(0x3d8A, 0x72); // OTP end address
	ov8856_write_i2c(0x3d8B, 0x0E);
	ov8856_write_i2c(0x3d81, 0x01); // load otp into buffer
	mdelay(10);

	// OTP base information and WB calibration data
	otp_flag = ov8856_read_i2c(0x7010);
	LOG_INF("ov8856_otp_read_test  otp_flag: 0x%x,\n", otp_flag); 
	addr = 0;

	if((otp_flag & 0xc0) == 0x40) 
	{
		addr = 0x7011; // base address of info group 1
		otp_ov8856_af_data[0]=((ov8856_read_i2c(0x7022) << 2) |((ov8856_read_i2c(0x7024)>>6)&0x03)) ;
		otp_ov8856_af_data[1]=((ov8856_read_i2c(0x7023) << 2) |((ov8856_read_i2c(0x7024)>>4)&0x03));
		
	}
	else if((otp_flag & 0x30) == 0x10) 
	{
		addr = 0x7019; // base address of info group 2
		otp_ov8856_af_data[0]=((ov8856_read_i2c(0x7025) << 2) |((ov8856_read_i2c(0x7027)>>6)&0x03)) ;
		otp_ov8856_af_data[1]=((ov8856_read_i2c(0x7026) << 2) |((ov8856_read_i2c(0x7027)>>4)&0x03));
	}
	if(addr != 0)
	{
		otp_info.flag = 0x40; // valid info and AWB in OTP
		otp_info.module_integrator_id = ov8856_read_i2c(addr);
		otp_info.lens_id = ov8856_read_i2c( addr + 1); 
		otp_info.production_year = ov8856_read_i2c( addr + 2); 
		otp_info.production_month = ov8856_read_i2c( addr + 3); 
		otp_info.production_day = ov8856_read_i2c(addr + 4); 
		temp = ov8856_read_i2c(addr + 7);
		otp_info.rg_ratio = (ov8856_read_i2c(addr + 5)<<2) + ((temp>>6) & 0x03);
		otp_info.bg_ratio = (ov8856_read_i2c(addr + 6)<<2) + ((temp>>4) & 0x03);
		
	}
	else
	{
		otp_info.flag = 0x00; // not info and AWB in OTP
		otp_info.module_integrator_id = 0;
		otp_info.lens_id = 0;
		otp_info.production_year = 0;
		otp_info.production_month = 0;
		otp_info.production_day = 0;
		otp_info.rg_ratio = 0;
		otp_info.bg_ratio = 0;
	}

	// OTP Lenc Calibration
	otp_flag = ov8856_read_i2c(0x7028);
	LOG_INF("ov8856_otp_read_test lsc otp_flag: 0x%x,\n", otp_flag); 

	addr = 0;
	int checksum2=0;	

	if((otp_flag & 0xc0) == 0x40)
	{
		addr = 0x7029; //base address of Lenc Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10)
	{
		addr = 0x711a;; //base address of Lenc Calibration group 2
	}	

	if(addr != 0)
	{
		for(i=0;i<240;i++)
		{
			otp_info.lenc[i]=ov8856_read_i2c(addr + i);
			checksum2 += otp_info.lenc[i];
		}
		checksum2 = (checksum2)%255 +1;
		otp_info.checksum = ov8856_read_i2c((addr + 240));

		if(otp_info.checksum == checksum2)
		{
			otp_info.flag |= 0x10;
		}
	}
	else
	{
		for(i=0;i<240;i++)
		{
			otp_info.lenc[i]=0;
		}
	}
	for(i=0x7010;i<=0x720e;i++) 
	{
		ov8856_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
	}
	//set 0x5001[3] to \A1\B01\A1\B1
	temp1 = ov8856_read_i2c(0x5001);
	ov8856_write_i2c(0x5001, (0x08 & 0x08) | (temp1 & (~0x08)));

	return otp_info.flag;
}

int ov8856_apply_otp(void)
{
	int rg, bg, R_gain, G_gain, B_gain, Base_gain, temp, i;
	// apply OTP WB Calibration
	if (otp_info.flag & 0x40)
	{
		rg = otp_info.rg_ratio;
		bg = otp_info.bg_ratio;
		//calculate G gain
		R_gain = (RG_Ratio_Typical*1000) / rg;
		B_gain = (BG_Ratio_Typical*1000) / bg;
		G_gain = 1000;
		if (R_gain < 1000 || B_gain < 1000)
		{
			if (R_gain < B_gain)
				Base_gain = R_gain;
			else
				Base_gain = B_gain;
		}
		else
		{
			Base_gain = G_gain;
		}
		R_gain = 0x400 * R_gain / (Base_gain);
		B_gain = 0x400 * B_gain / (Base_gain);
		G_gain = 0x400 * G_gain / (Base_gain);
		// update sensor WB gain
		if (R_gain>0x400)
		{
			ov8856_write_i2c(0x5019, R_gain>>8);
			ov8856_write_i2c(0x501a, R_gain & 0x00ff);
		}
		if (G_gain>0x400)
		{
			ov8856_write_i2c(0x501b, G_gain>>8);
			ov8856_write_i2c(0x501c, G_gain & 0x00ff);
		}
		if (B_gain>0x400)
		{
			ov8856_write_i2c(0x501d, B_gain>>8);
			ov8856_write_i2c(0x501e, B_gain & 0x00ff);
		}
	}
	printk("ov8856_sunwin_otp>>>>0x%x>>>0x%x>>>0x%x\n",R_gain,B_gain,G_gain);
	// apply OTP Lenc Calibration
	if (otp_info.flag & 0x10)
	{
		temp = ov8856_read_i2c(0x5000);
		temp = 0x20 | temp;
		ov8856_write_i2c(0x5000, temp);
		for(i=0;i<240;i++)
		{
			ov8856_write_i2c(0x5900 + i, otp_info.lenc[i]);
		}
		printk("ov8856_sunwin_otp>>>>>>lsc success\n");
	}
	return otp_info.flag;
}

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 0
#define OV8856_DEVICE_ID 0x20
#define CAM_CAL_DEV_MAJOR_NUMBER 226

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "OV8856_SUNWIN_CAM_CAL_DRV"
//#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
/* fix warning MSG 
static unsigned short g_pu2Normal_i2c[] = {OV8856_DEVICE_ID , I2C_CLIENT_END};
static unsigned short g_u2Ignore = I2C_CLIENT_END;
static struct i2c_client_address_data g_stCAM_CAL_Addr_data = {
    .normal_i2c = g_pu2Normal_i2c,
    .probe = &g_u2Ignore,
    .ignore = &g_u2Ignore
}; */

static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x20>>1)}; //make dummy_eeprom co-exist

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
/*******************************************************************************
*
********************************************************************************/

u8 OV8856_CheckID[]= {0x10,0xcc,0x00,0x40,0x88};

static int selective_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{    	
    printk("[OV8856_CAM_CAL] selective_read_region offset =%d size %d data read = %d\n", offset,size, *data);

if(size ==2 ){
	if(offset == 7){
		memcpy((void *)data,(void *)&otp_ov8856_af_data[0],size);
		printk("otp_ov8856_af_data[0] = 0x%x\n", otp_ov8856_af_data[0]);
		
	}
	
	else if(offset == 9){		 
		memcpy((void *)data,(void *)&otp_ov8856_af_data[1],size);
		printk("otp_ov8856_af_data[1] = 0x%x\n",otp_ov8856_af_data[1]);

	}
}

if(size == 4){
		memcpy((void *)data,(void *)&OV8856_CheckID[1],size);

	}
	printk("+ls.test[OV8856_CAM_CAL]  data1 = %d\n",*(UINT32 *)data);
}

#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            LOG_INF("[OV8856_CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                LOG_INF("[OV8856_CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        LOG_INF("[OV8856_CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
//fix warning MSG     LOG_INF("[OV8856_CAM_CAL] init Working buffer address 0x%x  command is 0x%08x\n", pWorkingBuff, a_u4Command);

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        LOG_INF("[OV8856_CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {


		 case CAM_CALIOC_S_WRITE:
    LOG_INF("[LOG_INF] CAM_CALIOC_S_WRITE \n");        
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = 0;
           // i4RetValue=iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
#endif
            break;

	
      
        case CAM_CALIOC_G_READ:
            LOG_INF("[OV8856_CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif 
            LOG_INF("[OV8856_CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            LOG_INF("[OV8856_CAM_CAL] length %d \n", ptempbuf->u4Length);
          // LOG_INF("[OV8856_CAM_CAL] Before read Working buffer address 0x%x \n", pWorkingBuff);


if(ptempbuf->u4Length == 2){

       i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff, 0x20, ptempbuf->u4Length);

}else if(ptempbuf->u4Length == 4){

	   i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff, 0x20, ptempbuf->u4Length);
 }        
		
          // LOG_INF("[OV8856_CAM_CAL] After read Working buffer address 0x%x \n", pWorkingBuff);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            

            break;
        default :
      	     LOG_INF("[OV8856_CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        LOG_INF("[OV8856_CAM_CAL] to user length %d \n", ptempbuf->u4Length);
		
  

  
		if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            LOG_INF("[OV8856_CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    LOG_INF("[OV8856 CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

//#if defined(MT6572)
	// do nothing
//#else
    //if(TRUE != hwPowerOn(MT65XX_POWER_LDO_VCAMA, VOL_2800, "S24CS64A"))
    //{
    //    LOG_INF("[OV8856_CAM_CAL] Fail to enable analog gain\n");
    //    return -EIO;
    //}
//#endif	

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        LOG_INF("[OV8856_CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        LOG_INF("[OV8856_CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        LOG_INF("[OV8856_CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        LOG_INF("[OV8856_CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }
 
    CAM_CAL_class = class_create(THIS_MODULE, "OV8856_CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        LOG_INF("Unable to create class, err = %d\n", ret);
        return ret;
    }
	
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{

    return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    //i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{

   // i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);


	 int i4RetValue = 0;
    LOG_INF("[OV8856_CAM_CAL]\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   LOG_INF(" [OV8856_CAM_CAL] register char device failed!\n");
	   return i4RetValue;
	}
	LOG_INF(" [OV8856_CAM_CAL] Attached!! \n");

   
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        printk("failed to register CAM_CAL driver\n");
        return -ENODEV;
    }
    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        printk("failed to register CAM_CAL driver\n");
        return -ENODEV;
    }	
LOG_INF(" OV8856_CAM_CAL  Attached Pass !! \n");
    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);
