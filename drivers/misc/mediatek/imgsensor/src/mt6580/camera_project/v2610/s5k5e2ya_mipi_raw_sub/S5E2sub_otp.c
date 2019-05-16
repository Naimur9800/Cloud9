#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k5e2yasubmipiraw_Sensor.h"

struct Darling_S5K5E2SUB_otp_struct {
int flag; 
int MID;
int LID;
int RGr_ratio;
int BGr_ratio;
int GbGr_ratio;
int VCM_start;
int VCM_end;
} Darling_S5K5E2SUB_OTP;

#define RGr_ratio_Typical    0x2FD//0x2FD
#define BGr_ratio_Typical    0x25C//0x25C
#define GbGr_ratio_Typical  0x400



extern  void S5K5E2SUB_write_cmos_sensor(u16 addr, u32 para);
extern  kal_uint16 S5K5E2SUB_read_cmos_sensor(u32 addr);

void Darling_S5K5E2SUB_read_OTP(struct Darling_S5K5E2SUB_otp_struct *S5K5E2SUB_OTP)
{
     printk("[xucs]Darling_S5K5E2SUB_read_OTP start\n");
     S5K5E2SUB_write_cmos_sensor(0x0A00,0x04);
     S5K5E2SUB_write_cmos_sensor(0x0A02,0x02);
     S5K5E2SUB_write_cmos_sensor(0x0A00,0x01);
     mdelay(5);
     unsigned char val=0;
     val=S5K5E2SUB_read_cmos_sensor(0x0A04);//flag of info and awb
     printk("[xucs]The val of address 0x0A04 is %x\n",val);
	 if(val==0x40)
	 	{
	 	S5K5E2SUB_OTP->flag = 0x01;
		S5K5E2SUB_OTP->MID = S5K5E2SUB_read_cmos_sensor(0x0A05);
		S5K5E2SUB_OTP->LID = S5K5E2SUB_read_cmos_sensor(0x0A06);
		S5K5E2SUB_OTP->RGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A07)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A08);
		S5K5E2SUB_OTP->BGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A09)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A0A);
		S5K5E2SUB_OTP->GbGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A0B)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A0C);
	 	}
	 else if(val==0xD0)
	 	{
	 	S5K5E2SUB_OTP->flag = 0x01;
		S5K5E2SUB_OTP->MID = S5K5E2SUB_read_cmos_sensor(0x0A0E);
		S5K5E2SUB_OTP->LID = S5K5E2SUB_read_cmos_sensor(0x0A0F);
		S5K5E2SUB_OTP->RGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A10)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A11);
		S5K5E2SUB_OTP->BGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A12)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A13);
		S5K5E2SUB_OTP->GbGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A14)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A15);
	 	}
	 else if(val==0xF4)
	 	{
	 	S5K5E2SUB_OTP->flag = 0x01;
		S5K5E2SUB_OTP->MID = S5K5E2SUB_read_cmos_sensor(0x0A17);
		S5K5E2SUB_OTP->LID = S5K5E2SUB_read_cmos_sensor(0x0A18);
		S5K5E2SUB_OTP->RGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A19)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A1A);
		S5K5E2SUB_OTP->BGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A1B)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A1C);
		S5K5E2SUB_OTP->GbGr_ratio = (S5K5E2SUB_read_cmos_sensor(0x0A1D)<<8)+S5K5E2SUB_read_cmos_sensor(0x0A1E);
	 	}
	 else
	 	{
	 	S5K5E2SUB_OTP->flag=0x00;
		S5K5E2SUB_OTP->MID =0x00;
		S5K5E2SUB_OTP->LID = 0x00;
		S5K5E2SUB_OTP->RGr_ratio = 0x00;
		S5K5E2SUB_OTP->BGr_ratio = 0x00;
		S5K5E2SUB_OTP->GbGr_ratio = 0x00;
	 	}
        S5K5E2SUB_write_cmos_sensor(0x0A00,0x04);
	    S5K5E2SUB_write_cmos_sensor(0x0A00,0x00);
	    printk("[xucs]The val of S5K5E2SUB_OTP->flag is %x\n",S5K5E2SUB_OTP->flag);
        printk("[xucs]Darling_S5K5E2SUB_read_OTP end\n");
		printk("RGr_ratio= %x\n",S5K5E2SUB_OTP->RGr_ratio);//jindy
	    printk("BGr_ratio= %x\n",S5K5E2SUB_OTP->BGr_ratio); //jindy
	    printk("GbGr_ratio= %x\n",S5K5E2SUB_OTP->GbGr_ratio); //jindy
	    printk(" RGr_ratio_Typical= %x\n", RGr_ratio_Typical); //jindy
        printk("BGr_ratio_Typical= %x\n", BGr_ratio_Typical); //jindy
        printk("GbGr_ratio_Typical= %x\n",GbGr_ratio_Typical); //jindy
}

void Darling_S5K5E2SUB_apply_OTP(struct Darling_S5K5E2SUB_otp_struct *S5K5E2SUB_OTP)
{
   printk("[xucs]Darling_S5K5E2SUB_apply_OTP start\n");
   printk("[xucs]S5K5E2SUB_OTP->flag is %d\n",S5K5E2SUB_OTP->flag);
   if(((S5K5E2SUB_OTP->flag)&0x03)!=0x01) 	return;
   
   	int R_gain,B_gain,Gb_gain,Gr_gain,Base_gain;
	R_gain = (RGr_ratio_Typical*1000)/S5K5E2SUB_OTP->RGr_ratio;
	B_gain = (BGr_ratio_Typical*1000)/S5K5E2SUB_OTP->BGr_ratio;
	Gb_gain = (GbGr_ratio_Typical*1000)/S5K5E2SUB_OTP->GbGr_ratio;
	Gr_gain = 1000;
       Base_gain = R_gain;
	if(Base_gain>B_gain) Base_gain=B_gain;
	if(Base_gain>Gb_gain) Base_gain=Gb_gain;
	if(Base_gain>Gr_gain) Base_gain=Gr_gain;
	R_gain = 0x100 * R_gain /Base_gain;
	B_gain = 0x100 * B_gain /Base_gain;
	Gb_gain = 0x100 * Gb_gain /Base_gain;
	Gr_gain = 0x100 * Gr_gain /Base_gain;
	if(Gr_gain>0x100)
		{
		     S5K5E2SUB_write_cmos_sensor(0x020e,Gr_gain>>8);
                   S5K5E2SUB_write_cmos_sensor(0x020f,Gr_gain&0xff);
		}
	if(R_gain>0x100)
		{
		     S5K5E2SUB_write_cmos_sensor(0x0210,R_gain>>8);
                   S5K5E2SUB_write_cmos_sensor(0x0211,R_gain&0xff);
		}
	if(B_gain>0x100)
		{
		     S5K5E2SUB_write_cmos_sensor(0x0212,B_gain>>8);
                   S5K5E2SUB_write_cmos_sensor(0x0213,B_gain&0xff);
		}
	if(Gb_gain>0x100)
		{
		     S5K5E2SUB_write_cmos_sensor(0x0214,Gb_gain>>8);
                   S5K5E2SUB_write_cmos_sensor(0x0215,Gb_gain&0xff);
		}
     printk("[xucs]Darling_S5K5E2SUB_apply_OTP end\n");
}
