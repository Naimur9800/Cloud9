/*Transsion Top Secret*/
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

//add by yinpeng.zhang for fingerprint power on start
#include <linux/regulator/consumer.h>
struct regulator *reg = NULL;
//add by yinpeng.zhang for fingerprint power on end
/*GPIO pins reference.*/
int gf_parse_dts(struct gf_dev* gf_dev)
{
    struct device_node *node;
    struct platform_device *pdev = NULL;
    int ret = 0;

    node = of_find_compatible_node(NULL, NULL, "mediatek,mt6753-fingerprint");
    if (node) {
        pdev = of_find_device_by_node(node);
        if (pdev) {
            gf_dev->pinctrl_gpios = devm_pinctrl_get(&pdev->dev);
            if (IS_ERR(gf_dev->pinctrl_gpios)) {
                ret = PTR_ERR(gf_dev->pinctrl_gpios);
                pr_info("%s can't find fingerprint pinctrl\n", __func__);
                return ret;
            }
        } else {
            pr_info("%s platform device is null\n", __func__);
        }
    } else {
        pr_info("%s device node is null\n", __func__);
    }

    gf_dev->pins_irq = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fingerprint_pin_irq");
    if (IS_ERR(gf_dev->pins_irq)) {
        ret = PTR_ERR(gf_dev->pins_irq);
        pr_info("%s can't find fingerprint pinctrl irq\n", __func__);
        return ret;
    }
	gf_dev->irq_pulldown= pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fingerprint_eint_pull_down");
    if (IS_ERR(gf_dev->irq_pulldown)) {
        ret = PTR_ERR(gf_dev->irq_pulldown);
        pr_info("%s can't find fingerprint pinctrl irq_pulldown\n", __func__);
        return ret;
    }
    gf_dev->pins_reset_high = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fingerprint_reset_high");
    if (IS_ERR(gf_dev->pins_reset_high)) {
        ret = PTR_ERR(gf_dev->pins_reset_high);
        pr_info("%s can't find fingerprint pinctrl reset_high\n", __func__);
        return ret;
    }
    gf_dev->pins_reset_low = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fingerprint_reset_low");
    if (IS_ERR(gf_dev->pins_reset_low)) {
        ret = PTR_ERR(gf_dev->pins_reset_low);
        pr_info("%s can't find fingerprint pinctrl reset_low\n", __func__);
        return ret;
    }
/*
	gf_dev->pins_power_high = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fingerprint_power_high");
    if (IS_ERR(gf_dev->pins_power_high)) {
        ret = PTR_ERR(gf_dev->pins_power_high);
        pr_info("%s can't find fingerprint pinctrl pins_power_high\n", __func__);
        return ret;
    }
    gf_dev->pins_power_low = pinctrl_lookup_state(gf_dev->pinctrl_gpios, "fingerprint_power_low");
    if (IS_ERR(gf_dev->pins_power_low)) {
        ret = PTR_ERR(gf_dev->pins_power_low);
        pr_info("%s can't find fingerprint pinctrl pins_power_low\n", __func__);
        return ret;
    }
*/
    pr_info("%s, get pinctrl success!\n", __func__);
    return 0;
}

void gf_cleanup(struct gf_dev* gf_dev)
{
    pr_info("[info] %s\n",__func__);
    if (gpio_is_valid(gf_dev->irq_gpio))
    {
        gpio_free(gf_dev->irq_gpio);
        pr_info("remove irq_gpio success\n");
    }
    if (gpio_is_valid(gf_dev->reset_gpio))
    {
        gpio_free(gf_dev->reset_gpio);
        pr_info("remove reset_gpio success\n");
    }
}


/*power management*/
int gf_power_on(struct gf_dev* gf_dev)
{
    int rc = 0;
	int ret = 0;
	pr_info("[info] %s entry \n",__func__);
	
//add by yinpeng.zhang for fingerprint poweron start	
	reg = regulator_get(&gf_dev->spi->dev, "vgp1");
	ret = regulator_set_voltage(reg, 2800000, 2800000);
	if (ret) {
		pr_info("regulator_set_voltage(%d) failed!\n", ret);
	}
	ret = regulator_enable(reg);
	if (ret){
		pr_info("regulator_enable() failed!\n");
	}
//add by yinpeng.zhang for fingerprint poweron end

//	hwPowerOn(MT6331_POWER_LDO_VIBR, VOL_2800, "fingerprint");
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->irq_pulldown);
	/*Reset GPIO Output-low before poweron*/
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_low);
	msleep(5);
	/*power on*/
//	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_power_high);
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_irq);
	msleep(10);
	/*Reset GPIO Output-high, GF works*/
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
	pr_info("[info] %s exit \n",__func__);
    return rc;
}

int gf_power_off(struct gf_dev* gf_dev)
{
    int rc = 0;	
	int ret = 0;
	pr_info("[info] %s entry \n",__func__);
//add by yinpeng.zhang for fingerprint poweron start	
	reg = regulator_get(&gf_dev->spi->dev, "vgp1");
	ret = regulator_disable(reg);
	if (ret)
		pr_info("regulator_disable() failed!\n");
//add by yinpeng.zhang for fingerprint poweron end
	
//	hwPowerDown(MT6331_POWER_LDO_VIBR, "fingerprint");
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->irq_pulldown);
	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_low);
	msleep(10);
//	pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_power_low);
//	msleep(2);
	pr_info("[info] %s exit \n",__func__);
    return rc;
}


/********************************************************************
*CPU output low level in RST pin to reset GF. This is the MUST action for GF.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
    pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_low);
    mdelay((delay_ms > 3)?delay_ms:3);
    pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_reset_high);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
    struct device_node *node;

    pinctrl_select_state(gf_dev->pinctrl_gpios, gf_dev->pins_irq);

    node = of_find_compatible_node(NULL, NULL, "mediatek,mt6753-fingerprint");
    if (node) {
        gf_dev->irq_num = irq_of_parse_and_map(node, 0);
        pr_info("%s, gf_irq = %d\n", __func__, gf_dev->irq_num);
        gf_dev->irq = gf_dev->irq_num;
    } else
        pr_info("%s can't find compatible node\n", __func__);
    return gf_dev->irq;
}

