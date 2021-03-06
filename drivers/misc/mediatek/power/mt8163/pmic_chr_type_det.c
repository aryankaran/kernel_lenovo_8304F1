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

#include <linux/delay.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/battery_common.h>

/* ============================================================*/
/*extern function*/
/* ============================================================*/
void __attribute__((weak)) Charger_Detect_Init(void)
{
	pr_notice("Charger_Detect_Init not be porting in USB driver\n");
}

void __attribute__((weak)) Charger_Detect_Release(void)
{
	pr_notice("Charger_Detect_Release not be porting in USB driver\n");
}

typedef enum {
   ACA_UNKNOWN = 0,
   ACA_DOCK, //has external power source
   ACA_A,     //B device on accessory port
   ACA_B,     //only for charging
   ACA_C,    // A device on accessory port
   ACA_UNCONFIG = 0x8, // Unconfig State
} ACA_TYPE;

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);

#define voltage_threshold 800//mv
#define voltage_otg 200

static int g_aca_detection = 0xF;

void hw_otg_reset_aca_type(void)
{
	g_aca_detection = 0xF;
	printk("%s!\r\n", __func__);
}

int hw_otg_get_aca_type_detection(void)
{
   int ADC_voltage, data[4], i, ret_value = 0;
   int times=1, IDDIG_CHANNEL_NUM = 1; //(AUXIN1)
   if( IMM_IsAdcInitReady() == 0 )
   {
	   printk("[%s]: AUXADC is not ready\n", __func__);
	   return ACA_UNCONFIG;
   }

   i = times;
   while (i--)
   {
		ret_value = IMM_GetOneChannelValue(IDDIG_CHANNEL_NUM, data, &ADC_voltage);
		ADC_voltage = ADC_voltage * 1500 / 4096;
		if (ret_value == -1) // AUXADC is busy
		{
			printk("IMM_GetOneChannelValue wrong, AUXADC is busy");
			ADC_voltage = 0;
		}
		printk("[%s]: AUXADC Channel %d, ADC_voltage %d\n, voltage_threshold %d", __func__, IDDIG_CHANNEL_NUM, ADC_voltage, voltage_threshold);
   }

	if (ADC_voltage > voltage_threshold)
		return ACA_UNCONFIG;
	else if (ADC_voltage < voltage_threshold && ADC_voltage > voltage_otg)
		return ACA_C;
	else
		return ACA_UNKNOWN;
}

int hw_otg_get_aca_type(void)
{
  /* need otg type detection */
  if(0xF == g_aca_detection)
	  g_aca_detection = hw_otg_get_aca_type_detection();

  return g_aca_detection;
}

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)

int hw_charging_get_charger_type(void)
{
	return STANDARD_HOST;
}

#else

static void hw_bc11_init(void)
{
	Charger_Detect_Init();

	/*RG_BC11_BIAS_EN=1*/
	upmu_set_rg_bc11_bias_en(0x1);
	/*RG_BC11_VSRC_EN[1:0]=00*/
	upmu_set_rg_bc11_vsrc_en(0x0);
	/*RG_BC11_VREF_VTH = [1:0]=00*/
	upmu_set_rg_bc11_vref_vth(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);
	/*RG_BC11_IPU_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipu_en(0x0);
	/*RG_BC11_IPD_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipd_en(0x0);
	/*BC11_RST=1*/
	upmu_set_rg_bc11_rst(0x1);
	/*BC11_BB_CTRL=1*/
	upmu_set_rg_bc11_bb_ctrl(0x1);
	/*msleep(10);*/
	mdelay(50);
}

static unsigned int hw_bc11_DCD(void)
{
	unsigned int wChargerAvail = 0;

	/*RG_BC11_IPU_EN[1.0] = 10*/
	upmu_set_rg_bc11_ipu_en(0x2);
	/*RG_BC11_IPD_EN[1.0] = 01*/
	upmu_set_rg_bc11_ipd_en(0x1);
	/*RG_BC11_VREF_VTH = [1:0]=01*/
	upmu_set_rg_bc11_vref_vth(0x1);
	/*RG_BC11_CMP_EN[1.0] = 10*/
	upmu_set_rg_bc11_cmp_en(0x2);
	/*msleep(20);*/
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	/*RG_BC11_IPU_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipu_en(0x0);
	/*RG_BC11_IPD_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipd_en(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);
	/*RG_BC11_VREF_VTH = [1:0]=00*/
	upmu_set_rg_bc11_vref_vth(0x0);

	return wChargerAvail;
}

/*****
static unsigned int hw_bc11_stepA1(void)
{
unsigned int wChargerAvail = 0;

upmu_set_rg_bc11_ipu_en(0x2);
upmu_set_rg_bc11_vref_vth(0x2);
upmu_set_rg_bc11_cmp_en(0x2);
mdelay(80);
wChargerAvail = upmu_get_rgs_bc11_cmp_out();
upmu_set_rg_bc11_ipu_en(0x0);
upmu_set_rg_bc11_cmp_en(0x0);

return  wChargerAvail;
}
*****/

static unsigned int hw_bc11_stepA2(void)
{
	unsigned int wChargerAvail = 0;

	/*RG_BC11_VSRC_EN[1.0] = 10*/
	upmu_set_rg_bc11_vsrc_en(0x2);
	/*RG_BC11_IPD_EN[1:0] = 01*/
	upmu_set_rg_bc11_ipd_en(0x1);
	/*RG_BC11_VREF_VTH = [1:0]=00*/
	upmu_set_rg_bc11_vref_vth(0x0);
	/*RG_BC11_CMP_EN[1.0] = 01*/
	upmu_set_rg_bc11_cmp_en(0x1);

	/*msleep(80);*/
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	/*RG_BC11_VSRC_EN[1:0]=00*/
	upmu_set_rg_bc11_vsrc_en(0x0);
	/*RG_BC11_IPD_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipd_en(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);

	return  wChargerAvail;
}

static unsigned int hw_bc11_stepB2(void)
{
	unsigned int wChargerAvail = 0;

	/*RG_BC11_IPU_EN[1:0]=10*/
	upmu_set_rg_bc11_ipu_en(0x2);
	/*RG_BC11_VREF_VTH = [1:0]=10*/
	upmu_set_rg_bc11_vref_vth(0x1);
	/*RG_BC11_CMP_EN[1.0] = 01*/
	upmu_set_rg_bc11_cmp_en(0x1);

	/*msleep(80);*/
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	/*RG_BC11_IPU_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipu_en(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);
	/*RG_BC11_VREF_VTH = [1:0]=00*/
	upmu_set_rg_bc11_vref_vth(0x0);

	return  wChargerAvail;
}

static void hw_bc11_done(void)
{
	/*RG_BC11_VSRC_EN[1:0]=00*/
	upmu_set_rg_bc11_vsrc_en(0x0);
	/*RG_BC11_VREF_VTH = [1:0]=0*/
	upmu_set_rg_bc11_vref_vth(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);
	/*RG_BC11_IPU_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipu_en(0x0);
	/*RG_BC11_IPD_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipd_en(0x0);
	/*RG_BC11_BIAS_EN=0*/
	upmu_set_rg_bc11_bias_en(0x0);

	Charger_Detect_Release();
}

int hw_charging_get_charger_type(void)
{
	CHARGER_TYPE ret = CHARGER_UNKNOWN;

	hw_bc11_init();

	if (1 == hw_bc11_DCD()) {
				ret = NONSTANDARD_CHARGER;
	} else {
		if (1 == hw_bc11_stepA2()) {
			if (1 == hw_bc11_stepB2())
				ret = STANDARD_CHARGER;
			else
				ret = CHARGING_HOST;
		} else {
			ret = STANDARD_HOST;
		}
	}
	hw_bc11_done();
	battery_log(BAT_LOG_CRTI, "CHR_Type_num = %d\r\n", ret);
	return ret;
}
#endif
