#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>


#include "tpd.h"

/* #define TIMER_DEBUG */


#ifdef TIMER_DEBUG
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#endif

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "tpd_config.h"






#ifdef  CONFIG_TOUCHSCREEN_FT6X05_DISABLE_KEY_WHEN_SLIDE
#include <linux/timer.h>

#define KEY_STORE_MAX 20
struct keystorevalue{
    	unsigned int   code;
	unsigned int   value;
//	unsigned long time;
};

struct keystore{
    	struct keystorevalue value[KEY_STORE_MAX];
	unsigned int    head;
	unsigned int    tail;
	unsigned char tp_key_init_flag;
	struct timer_list tp_key_delay_timer;
	struct work_struct tp_queue;	
	struct workqueue_struct *tp_key_wq;
	struct mutex tp_key_mutex;
	struct input_dev *dev;
};

static struct keystore *tp_keystore;

static unsigned long curtime=0;

#define Is_tp_buf_empty ((tp_keystore->head==tp_keystore->tail)?1:0)
#define Is_tp_buf_full ((((tp_keystore->head+1)%KEY_STORE_MAX)==tp_keystore->tail)?1:0)
static void tp_buf_clear(void)
{
	tp_keystore->head=tp_keystore->tail=0;
}

static int tp_buf_put(int keycode,int keyvalue)
{
	int ret=0;
	CTP_DBG("head=%d,tail=%d!\r\n",tp_keystore->head,tp_keystore->tail);
	mutex_lock(&tp_keystore->tp_key_mutex);
	if(!Is_tp_buf_full){
		tp_keystore->value[tp_keystore->head].code=keycode;
		tp_keystore->value[tp_keystore->head].value=keyvalue;
		tp_keystore->head=(tp_keystore->head+1)%KEY_STORE_MAX;
	}
	else
		ret=-1;
	mutex_unlock(&tp_keystore->tp_key_mutex);
	CTP_DBG("1 head=%d,tail=%d!\r\n",tp_keystore->head,tp_keystore->tail);
	return ret;
}

static int tp_buf_get(struct keystorevalue * key)
{
	int ret=0;
	mutex_lock(&tp_keystore->tp_key_mutex);
	if(!Is_tp_buf_empty){
		key->code=tp_keystore->value[tp_keystore->tail].code;
		key->value=tp_keystore->value[tp_keystore->tail].value;
		tp_keystore->tail=(tp_keystore->tail+1)%KEY_STORE_MAX;
	}
	else
		ret=-1;
	mutex_unlock(&tp_keystore->tp_key_mutex);
	return ret;
}

static int tp_key_handle(int keycode,int keyvalue)
{
	int ret =0;
	
	
	
	if(Is_tp_buf_empty){
		 CTP_DBG("tp mod_timer!\r\n");
		mod_timer(&tp_keystore->tp_key_delay_timer,jiffies + HZ/5);
	}
	tp_buf_put(keycode,keyvalue);
	
	
	return ret;
}

static void tp_delay_process(struct work_struct *work)
{
      struct keystorevalue  key;
    	 CTP_DBG("tp_delay_process!\r\n");
	 while(!tp_buf_get(&key)){
	 	CTP_DBG("key code=%d,value=%d!\r\n",key.code,key.value);
		
	 	input_report_key( tp_keystore->dev, key.code, key.value);
		input_sync(tp_keystore->dev);
		msleep(100);
	 }
}

static void tp_key_delay_timer_handler(unsigned long dev_addr)
{
	
      queue_work(tp_keystore->tp_key_wq, &tp_keystore->tp_queue);
}

int fts_6x06_key_cancel(void)
{
	CTP_DBG();
	tp_buf_clear();
	if(tp_keystore->tp_key_init_flag){
		cancel_work_sync(&tp_keystore->tp_queue);
		del_timer(&tp_keystore->tp_key_delay_timer);
	}
	return 0;
}
static int tp_key_exit(void)
{
	CTP_DBG();
	fts_6x06_key_cancel();
	if (tp_keystore->tp_key_wq)
		destroy_workqueue(tp_keystore->tp_key_wq);
	kfree(tp_keystore);
	tp_keystore->tp_key_init_flag=0;
	return 0;
}

static  unsigned long get_time (void)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	return (1000000 * tv.tv_sec + tv.tv_usec);
}
#endif