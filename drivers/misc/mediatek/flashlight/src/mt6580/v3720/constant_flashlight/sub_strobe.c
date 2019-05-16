/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock_sub); /* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;
static struct work_struct workTimeOut_sub;

//#define GPIO_ENF GPIO_CAMERA_FLASH_EN_PIN
//#define GPIO_ENT GPIO_CAMERA_FLASH_MODE_PIN
/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */

static int g_bLtVersion;

/*****************************************************************************
Functions
*****************************************************************************/
extern struct pinctrl *flashlightpinctrl;
extern struct pinctrl_state *flashlight_ext1_h ;
extern struct pinctrl_state *flashlight_ext1_l ;
extern struct pinctrl_state *flashlight_ext2_h;
extern struct pinctrl_state *flashlight_ext2_l ;
static void work_timeOutFunc(struct work_struct *data);

static int FL_Enable(void)
{
	PK_DBG(" FL_Enable sub line=%d\n",__LINE__);
	if(g_duty==0)
	{
		pinctrl_select_state(flashlightpinctrl, flashlight_ext2_h);
		pinctrl_select_state(flashlightpinctrl, flashlight_ext1_l);
	}
	else
	{
		pinctrl_select_state(flashlightpinctrl, flashlight_ext2_h);
		pinctrl_select_state(flashlightpinctrl, flashlight_ext1_h);
	}
	return 0;
}

static int FL_Disable(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_ext2_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_ext1_l);

	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	g_duty = duty;
	return 0;
}

static int FL_Init(void)
{

	pinctrl_select_state(flashlightpinctrl, flashlight_ext1_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_ext2_l);

	PK_DBG(" FL_Init line=%d\n",__LINE__);
	return 0;
}

static int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
  User interface
 *****************************************************************************/
static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut_sub);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;

static void timerInit(void)
{
	INIT_WORK(&workTimeOut_sub, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;
}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	PK_DBG("sub dummy ioctl");
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%ld\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
	switch(cmd)
	{

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
			break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {

			int s;
			int ms;

			if (g_timeOutTimeMs > 1000) {
				s = g_timeOutTimeMs / 1000;
				ms = g_timeOutTimeMs - s * 1000;
			} else {
				s = 0;
				ms = g_timeOutTimeMs;
			}

			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(s, ms * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
				}
				FL_Enable();
			}
			else
			{
				FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
			}
			break;
		default :
			PK_DBG(" No such command \n");
			i4RetValue = -EPERM;
			break;
	}
	return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
	PK_DBG("sub dummy open");
	int i4RetValue = 0;
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock_sub);

	if(strobe_Res)
	{
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	}
	else
	{
		strobe_Res += 1;
	}
	spin_unlock_irq(&g_strobeSMPLock_sub);

	return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
	PK_DBG("sub dummy release");

	if (strobe_Res)
	{
		spin_lock_irq(&g_strobeSMPLock_sub);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock_sub);

		FL_Uninit();
	}
	return 0;

}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}
