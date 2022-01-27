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
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif
#endif

#include "lcm_drv.h"
/*#include "ddp_irq.h"*/
#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)
#define REGFLAG_DELAY 0xFFFC
/* END OF REGISTERS MARKER */
#define REGFLAG_END_OF_TABLE 0xFFFD

struct regulator *lcm_vgp3;
#ifdef BUILD_LK

#ifdef GPIO_LCD_ENP
#define GPIO_LCD_ENP_EN      GPIO_LCD_ENP
#else
#define GPIO_LCD_ENP_EN      0xFFFFFFFF
#endif

#ifdef GPIO_LCD_ENN
#define GPIO_LCD_ENN_EN      GPIO_LCD_ENN
#else
#define GPIO_LCD_ENN_EN      0xFFFFFFFF
#endif


#ifdef GPIO_LCM_RST
#define GPIO_LCD_RST      GPIO_LCM_RST
#else
#define GPIO_LCD_RST      0xFFFFFFFF
#endif

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
#else

static struct regulator *lcm_vgp;
static unsigned int GPIO_LCD_ENP_EN;
static unsigned int GPIO_LCD_ENN_EN;
static unsigned int GPIO_LCD_RST_EN;

static void lcm_get_gpio_infor(struct device *dev)
{
	pr_err("xrd LCM: lcm_get_gpio_infor is going\n");

	GPIO_LCD_ENP_EN = of_get_named_gpio(dev->of_node, "lcm_enp_gpio", 0);
	gpio_request(GPIO_LCD_ENP_EN, "GPIO_LCD_ENP_EN");
	GPIO_LCD_ENN_EN = of_get_named_gpio(dev->of_node, "lcm_enn_gpio", 0);
	gpio_request(GPIO_LCD_ENN_EN, "GPIO_LCD_ENN_EN");
	GPIO_LCD_RST_EN = of_get_named_gpio(dev->of_node, "lcm_reset_gpio", 0);
	gpio_request(GPIO_LCD_RST_EN, "GPIO_LCD_RST_EN");
	pr_err("xrd LCM: GPIO_LCD_ENP_EN=%d,GPIO_LCD_ENN_EN=%d,GPIO_LCD_RST_EN=%d.\n",GPIO_LCD_ENP_EN,GPIO_LCD_ENN_EN,GPIO_LCD_RST_EN);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	pr_debug("LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;
	lcm_vgp3 =lcm_vgp;

	return ret;
}

static int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_debug("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp)
		return 0;

	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 1800000)
		pr_err("LCM: check regulator voltage=1800000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=1800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

static int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_debug("LCM: lcm query regulator enable status[0x%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}

	return ret;
}
static int lcm_driver_probe(struct device *dev, void const *data)
{
		pr_err("xrd LCM: lcm_driver_probe \n");
		lcm_get_vgp_supply(dev);
		lcm_get_gpio_infor(dev);

		return 0;
}
static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "lcm,lcm_dts",
		.data = 0,
	}, {
		/* sentinel */
	}
};


MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}



static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "JD9366_STARRY_DSI_VDO",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
		   },
};

static int __init lcm_init(void)
{
	pr_err("xrd LCM: lcm_init\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_err("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
#endif
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)							lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)							lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE	0
struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xE0,1,{0x00}},
{0xE1,1,{0x93}},
{0xE2,1,{0x65}},
{0xE3,1,{0xF8}},
{0x80,1,{0x03}},
{0xE0,1,{0x01}},
{0x00,1,{0x00}},
{0x17,1,{0x00}},
{0x18,1,{0xB1}},
{0x19,1,{0x01}},
{0x1A,1,{0x00}},
{0x1B,1,{0xB1}},
{0x1C,1,{0x01}},
{0x1F,1,{0x3E}},
{0x20,1,{0x28}},
{0x21,1,{0x28}},
{0x22,1,{0x0E}},
{0x37,1,{0x19}},
{0x38,1,{0x05}},
{0x39,1,{0x08}},
{0x3A,1,{0x12}},
{0x3C,1,{0x78}},
{0x3D,1,{0xFF}},
{0x3E,1,{0xFF}},
{0x3F,1,{0xFF}},
{0x40,1,{0x06}},
{0x41,1,{0xA0}},
{0x43,1,{0x08}},
{0x44,1,{0x07}},
{0x45,1,{0x40}},
{0x4B,1,{0x04}},
{0x55,1,{0x0F}},
{0x56,1,{0x01}},
{0x57,1,{0x89}},
{0x58,1,{0x0A}},
{0x59,1,{0x0A}},
{0x5A,1,{0x28}},
{0x5B,1,{0x14}},
{0x5D,1,{0x7C}}, //18
{0x5E,1,{0x66}}, //17
{0x5F,1,{0x57}}, //16
{0x60,1,{0x49}}, //15
{0x61,1,{0x46}}, //14
{0x62,1,{0x38}}, //13
{0x63,1,{0x3C}}, //12
{0x64,1,{0x25}}, //11
{0x65,1,{0x3E}}, //10
{0x66,1,{0x3B}}, //9
{0x67,1,{0x3A}}, //8
{0x68,1,{0x55}}, //7
{0x69,1,{0x40}}, //6
{0x6A,1,{0x47}}, //5
{0x6B,1,{0x38}}, //4
{0x6C,1,{0x31}}, //3
{0x6D,1,{0x22}}, //2
{0x6E,1,{0x11}}, //1
{0x6F,1,{0x00}}, //0
{0x70,1,{0x7C}}, //18
{0x71,1,{0x66}}, //17
{0x72,1,{0x57}}, //16
{0x73,1,{0x49}}, //15
{0x74,1,{0x46}}, //14
{0x75,1,{0x38}}, //13
{0x76,1,{0x3C}}, //12
{0x77,1,{0x25}}, //11
{0x78,1,{0x3E}}, //10
{0x79,1,{0x3B}}, //9
{0x7A,1,{0x3A}}, //8
{0x7B,1,{0x55}}, //7
{0x7C,1,{0x40}}, //6
{0x7D,1,{0x47}}, //5
{0x7E,1,{0x38}}, //4
{0x7F,1,{0x31}}, //3
{0x80,1,{0x22}}, //2
{0x81,1,{0x11}}, //1
{0x82,1,{0x00}}, //0
{0xE0,1,{0x02}},
{0x00,1,{0x47}},
{0x01,1,{0x47}},
{0x02,1,{0x45}},
{0x03,1,{0x45}},
{0x04,1,{0x4B}},
{0x05,1,{0x4B}},
{0x06,1,{0x49}},
{0x07,1,{0x49}},
{0x08,1,{0x41}},
{0x09,1,{0x1F}},
{0x0A,1,{0x1F}},
{0x0B,1,{0x1F}},
{0x0C,1,{0x1F}},
{0x0D,1,{0x1F}},
{0x0E,1,{0x1F}},
{0x0F,1,{0x43}},
{0x10,1,{0x1F}},
{0x11,1,{0x1F}},
{0x12,1,{0x1F}},
{0x13,1,{0x1F}},
{0x14,1,{0x1F}},
{0x15,1,{0x1F}},
{0x16,1,{0x46}},
{0x17,1,{0x46}},
{0x18,1,{0x44}},
{0x19,1,{0x44}},
{0x1A,1,{0x4A}},
{0x1B,1,{0x4A}},
{0x1C,1,{0x48}},
{0x1D,1,{0x48}},
{0x1E,1,{0x40}},
{0x1F,1,{0x1F}},
{0x20,1,{0x1F}},
{0x21,1,{0x1F}},
{0x22,1,{0x1F}},
{0x23,1,{0x1F}},
{0x24,1,{0x1F}},
{0x25,1,{0x42}},
{0x26,1,{0x1F}},
{0x27,1,{0x1F}},
{0x28,1,{0x1F}},
{0x29,1,{0x1F}},
{0x2A,1,{0x1F}},
{0x2B,1,{0x1F}},
{0x2C,1,{0x11}},
{0x2D,1,{0x0F}},
{0x2E,1,{0x0D}},
{0x2F,1,{0x0B}},
{0x30,1,{0x09}},
{0x31,1,{0x07}},
{0x32,1,{0x05}},
{0x33,1,{0x18}},
{0x34,1,{0x17}},
{0x35,1,{0x1F}},
{0x36,1,{0x01}},
{0x37,1,{0x1F}},
{0x38,1,{0x1F}},
{0x39,1,{0x1F}},
{0x3A,1,{0x1F}},
{0x3B,1,{0x1F}},
{0x3C,1,{0x1F}},
{0x3D,1,{0x1F}},
{0x3E,1,{0x1F}},
{0x3F,1,{0x13}},
{0x40,1,{0x1F}},
{0x41,1,{0x1F}},
{0x42,1,{0x10}},
{0x43,1,{0x0E}},
{0x44,1,{0x0C}},
{0x45,1,{0x0A}},
{0x46,1,{0x08}},
{0x47,1,{0x06}},
{0x48,1,{0x04}},
{0x49,1,{0x18}},
{0x4A,1,{0x17}},
{0x4B,1,{0x1F}},
{0x4C,1,{0x00}},
{0x4D,1,{0x1F}},
{0x4E,1,{0x1F}},
{0x4F,1,{0x1F}},
{0x50,1,{0x1F}},
{0x51,1,{0x1F}},
{0x52,1,{0x1F}},
{0x53,1,{0x1F}},
{0x54,1,{0x1F}},
{0x55,1,{0x12}},
{0x56,1,{0x1F}},
{0x57,1,{0x1F}},
{0x58,1,{0x40}},
{0x5B,1,{0x30}},
{0x5C,1,{0x02}},
{0x5D,1,{0x40}},
{0x5E,1,{0x01}},
{0x5F,1,{0x02}},
{0x63,1,{0x62}},
{0x64,1,{0x62}},
{0x67,1,{0x74}},
{0x68,1,{0x04}},
{0x69,1,{0x62}},
{0x6A,1,{0x66}},
{0x6B,1,{0x08}},
{0x6C,1,{0x00}},
{0x6D,1,{0x00}},
{0x6E,1,{0x00}},
{0x6F,1,{0x08}},
{0xE0,1,{0x03}},
{0x9B,1,{0x04}},
{0xAF,1,{0x20}},
{0xE0,1,{0x04}},
{0x09,1,{0x10}},
{0x0E,1,{0x48}},
{0x2B,1,{0x2B}},
{0x2E,1,{0x44}},
{0x41,1,{0xFF}},
{0x4B,1,{0xFF}},
{0xE0,1,{0x00}},
{0xE6,1,{0x02}},
{0xE7,1,{0x06}},
{0x51,1,{0xFF}},
{0x53,1,{0x2C}},
{0x55,1,{0x01}},
{0x11, 0, {} },
{REGFLAG_DELAY, 120, {} },
{0x29, 0, {} },
{REGFLAG_DELAY, 20, {} },
{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
#ifdef BUILD_LK
				dprintf(0, "[LK]REGFLAG_DELAY\n");
#endif
				if (table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_init_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init_power() enter\n");
	MDELAY(20);
	upmu_set_rg_vgp3_vosel(3);
	upmu_set_rg_vgp3_en(0x1);
#else
	pr_debug("[Kernel/LCM] lcm_init_power() enter\n");
#endif
}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend_power() enter\n");
	MDELAY(20);
	upmu_set_rg_vgp3_vosel(0);
	upmu_set_rg_vgp3_en(0x0);
#else
	pr_debug("[Kernel/LCM] lcm_suspend_power() enter\n");
	lcm_vgp_supply_disable();
	MDELAY(20);
#endif
}

static void lcm_resume_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume_power() enter\n");
	MDELAY(20);
	upmu_set_rg_vgp3_vosel(3);
	upmu_set_rg_vgp3_en(0x1);
#else
	pr_debug("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_vgp_supply_enable();
	MDELAY(20);
#endif
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

#define AUXADC_LCM_VOLTAGE_CHANNEL 15 //aux_in5
#define LCM_VOLTAGE_LEVEL 700 //TXD < 700; STARRY > 700
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static void lcm_get_params(LCM_PARAMS *params)
{
	int ret = 0, data[4], rawvalue, lcm_vol;
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
#endif

	/* DSI */
	/* Command mode setting */
	/* Three lane or Four lane */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	/* The following defined the format for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;

	/* Video mode setting */
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 4;
	params->dsi.vertical_frontporch = 8;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 48;
	params->dsi.horizontal_frontporch = 32;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.ssc_disable = 1;
	params->dsi.PLL_CLOCK = 250;
	params->dsi.cont_clock = 0;
	params->dsi.clk_lp_per_line_enable=1;

	ret = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL, data, &rawvalue);
	if(ret)
	{
		IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL, data, &rawvalue);
	}
	lcm_vol = rawvalue * 1800 / 4096;
	if(lcm_vol>LCM_VOLTAGE_LEVEL)
	{
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	pr_err("[Kernel/LCM]JD9366_STARRY ENABLE LCM ESD\n");
	}
}

static void lcm_init_lcm(void)
{

#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init() enter\n");
	lcm_set_gpio_output(GPIO_LCD_RST, 1);
	MDELAY(20);

	lcm_set_gpio_output(GPIO_LCD_RST, 0);
	MDELAY(20);

	lcm_set_gpio_output(GPIO_LCD_RST, 1);
	MDELAY(20);
#else
  pr_err("[Kernel/LCM] STARRY lcm_init() enter\n");
	lcm_vgp_supply_enable();
	lcm_set_gpio_output(GPIO_LCD_ENP_EN, 1);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_ENN_EN, 1);
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 0);
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	MDELAY(20);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

#endif

}

void lcm_suspend(void)
{

#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend() enter\n");

	lcm_set_gpio_output(GPIO_LCD_RST, 1);
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_RST, 0);
	MDELAY(10);
#else
	pr_debug("[Kernel/LCM] lcm_suspend() enter\n");
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_RST_EN, 0);
	MDELAY(10);
		
	lcm_set_gpio_output(GPIO_LCD_ENN_EN, 0);
	lcm_set_gpio_output(GPIO_LCD_ENP_EN, 0);
	MDELAY(5);
#endif

}

void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume() enter\n");
#else
	pr_debug("[Kernel/LCM] lcm_resume() enter\n");
	lcm_set_gpio_output(GPIO_LCD_ENP_EN, 1);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_ENN_EN, 1);
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 0);
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_RST_EN, 1);
	MDELAY(20);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	pr_err("[Kernel/LCM] lcm_resume() end\n");
#endif
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00290508;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

static unsigned int lcm_compare_id(void)
{
#ifdef BUILD_LK 
	unsigned int id = 0;
	unsigned char buffer[3];
	unsigned int data_array[16];
	//VDD power on ->VGP3_PMU 1.8V
	upmu_set_rg_vgp3_vosel(3);
	upmu_set_rg_vgp3_en(0x1);
	MDELAY(5);

	lcm_set_gpio_output(GPIO_LCD_ENP_EN, GPIO_OUT_ONE);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_ENN_EN, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(120);

	data_array[0]=0x00053902;
	data_array[1]=0x659300E0;
	data_array[2]=0x000000F8;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10);
	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, buffer, 3);
	id = (buffer[0] << 8) | buffer [1]; 
	
	#ifdef BUILD_LK
		dprintf(0, "[LK/LCM] lcm_compare_id() lcm_id=%x\n",id);
    #else        
		printk("[Kernel/LCM] lcm_compare_id() lcm_id=%x\n",id);
	#endif
	return (id == 0x9366)?1:0;
#else 
	printk("[Kernel/LCM] lcm_compare_id()\n");
	return 1;
#endif
}

LCM_DRIVER jd9366_starry_dsi_vdo_lcm_drv =
{
	.name = "JD9366_STARRY_DSI_VDO",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
	.update = lcm_update,
#endif
};
