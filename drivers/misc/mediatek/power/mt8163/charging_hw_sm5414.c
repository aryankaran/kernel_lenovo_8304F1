#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>

#include <linux/delay.h>
//#include <mt-plat/mtk_boot.h>
//#include <mach/mtk_charging.h>
#include "sm5414.h"

//add android m api
#include <linux/types.h>
#include <mt-plat/battery_common.h>
//#include <mach/mtk_pmic.h>
#include <linux/reboot.h>
#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mach/mtk_diso.h>
#include <mt-plat/diso.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#ifdef MTK_DISCRETE_SWITCH
#include <mach/eint.h>
#include <cust_eint.h>
#include <mach/mtk_gpio.h>
#include <cust_gpio_usage.h>
#endif
#endif

#define STATUS_OK               0
#define STATUS_UNSUPPORTED      -1
#define GETARRAYNUM(array)      (sizeof(array)/sizeof(array[0]))

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#ifndef CUST_GPIO_VIN_SEL
#define CUST_GPIO_VIN_SEL       18
#endif
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#define SW_POLLING_PERIOD       100 //100 ms
#define MSEC_TO_NSEC(x)         (x * 1000000UL)

static DEFINE_MUTEX(diso_polling_mutex);
static DECLARE_WAIT_QUEUE_HEAD(diso_polling_thread_wq);
static struct hrtimer diso_kthread_timer;
static kal_bool diso_thread_timeout = KAL_FALSE;
static struct delayed_work diso_polling_work;
static void diso_polling_handler(struct work_struct *work);
static DISO_Polling_Data DISO_Polling;
#else
DISO_IRQ_Data DISO_IRQ;
#endif
int g_diso_state = 0;
int vin_sel_gpio_number = (CUST_GPIO_VIN_SEL | 0x80000000);

static char *DISO_state_s[8] = {
    "IDLE",
    "OTG_ONLY",
    "USB_ONLY",
    "USB_WITH_OTG",
    "DC_ONLY",
    "DC_WITH_OTG",
    "DC_WITH_USB",
    "DC_USB_OTG",
};
#endif

extern unsigned int upmu_get_reg_value(unsigned int reg);
extern bool mt_usb_is_device(void);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern int hw_charging_get_charger_type(void);
extern void mt_power_off(void);
extern unsigned int mt6311_get_chip_id(void);
extern int is_mt6311_exist(void);
extern int is_mt6311_sw_ready(void);


kal_bool charging_type_det_done = KAL_TRUE;

const unsigned int VBAT_CV_VTH[] = {
    BATTERY_VOLT_04_100000_V,   BATTERY_VOLT_04_125000_V,   BATTERY_VOLT_04_150000_V,   BATTERY_VOLT_04_175000_V,
    BATTERY_VOLT_04_200000_V,   BATTERY_VOLT_04_225000_V,   BATTERY_VOLT_04_250000_V,   BATTERY_VOLT_04_275000_V,
    BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_325000_V,   BATTERY_VOLT_04_350000_V,   BATTERY_VOLT_04_375000_V,
    BATTERY_VOLT_04_400000_V,   BATTERY_VOLT_04_425000_V,   BATTERY_VOLT_04_450000_V,   BATTERY_VOLT_04_475000_V,
};

const unsigned int CS_VTH[] = {
    CHARGE_CURRENT_100_00_MA,   CHARGE_CURRENT_150_00_MA,   CHARGE_CURRENT_200_00_MA, CHARGE_CURRENT_250_00_MA,
    CHARGE_CURRENT_300_00_MA,   CHARGE_CURRENT_350_00_MA,   CHARGE_CURRENT_400_00_MA, CHARGE_CURRENT_450_00_MA,
    CHARGE_CURRENT_500_00_MA,   CHARGE_CURRENT_550_00_MA,   CHARGE_CURRENT_600_00_MA, CHARGE_CURRENT_650_00_MA,
    CHARGE_CURRENT_700_00_MA,   CHARGE_CURRENT_750_00_MA,   CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_850_00_MA,
    CHARGE_CURRENT_900_00_MA,   CHARGE_CURRENT_950_00_MA,   CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1050_00_MA,
    CHARGE_CURRENT_1100_00_MA,   CHARGE_CURRENT_1150_00_MA,   CHARGE_CURRENT_1200_00_MA, CHARGE_CURRENT_1250_00_MA,
    CHARGE_CURRENT_1300_00_MA,   CHARGE_CURRENT_1350_00_MA,   CHARGE_CURRENT_1400_00_MA, CHARGE_CURRENT_1450_00_MA,
    CHARGE_CURRENT_1500_00_MA,   CHARGE_CURRENT_1550_00_MA,   CHARGE_CURRENT_1600_00_MA, CHARGE_CURRENT_1650_00_MA,
    CHARGE_CURRENT_1700_00_MA,   CHARGE_CURRENT_1750_00_MA,   CHARGE_CURRENT_1800_00_MA, CHARGE_CURRENT_1850_00_MA,
    CHARGE_CURRENT_1900_00_MA,   CHARGE_CURRENT_1950_00_MA,   CHARGE_CURRENT_2000_00_MA, CHARGE_CURRENT_2050_00_MA,
    CHARGE_CURRENT_2100_00_MA,   CHARGE_CURRENT_2150_00_MA,   CHARGE_CURRENT_2200_00_MA, CHARGE_CURRENT_2250_00_MA,
    CHARGE_CURRENT_2300_00_MA,   CHARGE_CURRENT_2350_00_MA,   CHARGE_CURRENT_2400_00_MA, CHARGE_CURRENT_2450_00_MA,
    CHARGE_CURRENT_2500_00_MA,
};

const unsigned int INPUT_CS_VTH[] = {
    CHARGE_CURRENT_100_00_MA,   CHARGE_CURRENT_150_00_MA,   CHARGE_CURRENT_200_00_MA, CHARGE_CURRENT_250_00_MA,
    CHARGE_CURRENT_300_00_MA,   CHARGE_CURRENT_350_00_MA,   CHARGE_CURRENT_400_00_MA, CHARGE_CURRENT_450_00_MA,
    CHARGE_CURRENT_500_00_MA,   CHARGE_CURRENT_550_00_MA,   CHARGE_CURRENT_600_00_MA, CHARGE_CURRENT_650_00_MA,
    CHARGE_CURRENT_700_00_MA,   CHARGE_CURRENT_750_00_MA,   CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_850_00_MA,
    CHARGE_CURRENT_900_00_MA,   CHARGE_CURRENT_950_00_MA,   CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1050_00_MA,
    CHARGE_CURRENT_1100_00_MA,   CHARGE_CURRENT_1150_00_MA,   CHARGE_CURRENT_1200_00_MA, CHARGE_CURRENT_1250_00_MA,
    CHARGE_CURRENT_1300_00_MA,   CHARGE_CURRENT_1350_00_MA,   CHARGE_CURRENT_1400_00_MA, CHARGE_CURRENT_1450_00_MA,
    CHARGE_CURRENT_1500_00_MA,   CHARGE_CURRENT_1550_00_MA,   CHARGE_CURRENT_1600_00_MA, CHARGE_CURRENT_1650_00_MA,
    CHARGE_CURRENT_1700_00_MA,   CHARGE_CURRENT_1750_00_MA,   CHARGE_CURRENT_1800_00_MA, CHARGE_CURRENT_1850_00_MA,
    CHARGE_CURRENT_1900_00_MA,   CHARGE_CURRENT_1950_00_MA,   CHARGE_CURRENT_2000_00_MA, CHARGE_CURRENT_2050_00_MA,
};

const unsigned int VCDT_HV_VTH[] = {
    BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,   BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
    BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,   BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
    BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,   BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
    BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,   BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V
};

static unsigned int charging_error = false;
//static unsigned int charging_get_error_state(void);
static unsigned int charging_set_error_state(void *data);

unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size, const unsigned int val)
{
    if (val < array_size) {
        return parameter[val];
    } else {
        printk("Can't find the parameter \r\n");
        return parameter[0];
    }
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size, const unsigned int val)
{
    unsigned int i;

    printk("array_size = %d \r\n", array_size);

    for(i = 0; i < array_size; i++) {
        if (val == *(parameter + i))
            return i;
    }

    printk("NO register value match. val=%d\r\n", val);

    return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList,unsigned int number,unsigned int level)
{
    unsigned int i;
    unsigned int max_value_in_last_element;

    if (pList[0] < pList[1]) {
        max_value_in_last_element = KAL_TRUE;
    } else {
        max_value_in_last_element = KAL_FALSE;
    }
    if (max_value_in_last_element == KAL_TRUE) {
        //max value in the last element
        for(i = (number-1); i != 0; i--) {
            if (pList[i] <= level)
                return pList[i];
        }

        printk("Can't find closest level, small value first \r\n");
        return pList[0];
    } else {
        // max value in the first element
        for(i = 0; i< number; i++) {
            if (pList[i] <= level)
                return pList[i];
        }

        printk("Can't find closest level, large value first \r\n");
        return pList[number -1];
    }
}

static unsigned int charging_hw_init(void *data)
{
    unsigned int status = STATUS_OK;
#if 0
    mt_set_gpio_mode(GPIO_SM5414_SHDN_PIN,GPIO_MODE_GPIO);
    mt_set_gpio_dir(GPIO_SM5414_SHDN_PIN,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_SM5414_SHDN_PIN,GPIO_OUT_ONE);
#endif

    sm5414_set_topoff(TOPOFF_100mA);
/*
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
    sm5414_set_batreg(BATREG_4_3_5_0_V); //VREG 4.352V
#else
    sm5414_set_batreg(BATREG_4_2_0_0_V); //VREG 4.208V
#endif
*/
#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
    mt_set_gpio_mode(vin_sel_gpio_number,0); // 0:GPIO mode
    mt_set_gpio_dir(vin_sel_gpio_number,0); // 0: input, 1: output
#endif

#if defined(SM5414_TOPOFF_TIMER_SUPPORT)
    sm5414_set_autostop(AUTOSTOP_EN);
    sm5414_set_topofftimer(TOPOFFTIMER_10MIN);
#else
    sm5414_set_autostop(AUTOSTOP_DIS);
#endif

    return status;
}

static unsigned int charging_dump_register(void *data)
{
    printk("charging_dump_register\r\n");

    sm5414_dump_register();

    return STATUS_OK;
}

extern int g_runin_battery_test_flag;
static unsigned int charging_enable(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int enable = *(unsigned int*)(data);

	printk("[charging_enable]-enable(%d)-\n",enable);

    if (KAL_TRUE == enable) 
	{
#if 0
        mt_set_gpio_mode(GPIO_SM5414_CHGEN_PIN,GPIO_MODE_GPIO);
        mt_set_gpio_dir(GPIO_SM5414_CHGEN_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN,GPIO_OUT_ZERO);

        silicon_sm5414_set_gpio(true);
#endif
        sm5414_set_suspen(SUSPEND_DIS);	/* enable power path */
        sm5414_set_chgen(CHARGE_EN);
        //pinctrl_select_state(g_sm5414_dev.pinctrl,g_sm5414_dev.sm5414_charge_enable);
    } 
	else
	{
#if defined(CONFIG_USB_MTK_HDRC_HCD)
        if (mt_usb_is_device())
#endif
        {
#if 0
            mt_set_gpio_mode(GPIO_SM5414_CHGEN_PIN,GPIO_MODE_GPIO);
            mt_set_gpio_dir(GPIO_SM5414_CHGEN_PIN,GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN,GPIO_OUT_ONE);

            silicon_sm5414_set_gpio(false);
#endif	
            sm5414_set_chgen(CHARGE_DIS);
            //pinctrl_select_state(g_sm5414_dev.pinctrl,g_sm5414_dev.sm5414_charge_disable);
			if (g_runin_battery_test_flag == 1)
				sm5414_set_suspen(SUSPEND_EN);	/* disable power path */
			else
				sm5414_set_suspen(SUSPEND_DIS); /* enable power path */
        }

#if 0//defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
        bq24296_set_chg_config(0x0);
        bq24296_set_en_hiz(0x1);    // disable power path
#endif
    }

    return status;
}

static unsigned int charging_set_cv_voltage(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int array_size;
    unsigned int set_cv_voltage;
    unsigned short register_value;
    unsigned int cv_value = *(unsigned int *)(data);

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
    if (cv_value >= BATTERY_VOLT_04_350000_V)
        cv_value = 4375000;//4350000;
#endif

    //use nearest value
    if (BATTERY_VOLT_04_200000_V == cv_value) {
        cv_value = 4208000;
    }
    array_size = GETARRAYNUM(VBAT_CV_VTH);
    set_cv_voltage = bmt_find_closest_level(VBAT_CV_VTH, array_size, cv_value);
    register_value = charging_parameter_to_value(VBAT_CV_VTH, array_size, set_cv_voltage);
    printk("[cgh][%s]register_value =%x\r\n",__func__,register_value);
    sm5414_set_batreg(register_value);

#if 0
    //for jeita recharging issue
    if (pre_register_value != register_value)
        bq24296_set_chg_config(1);

    pre_register_value = register_value;
#endif
    return status;
}

static unsigned int charging_get_current(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int array_size;
    unsigned char reg_value;
#if 0
    unsigned char ret_val = 0;
    unsigned char ret_force_20pct = 0;

    //Get current level
    bq24296_read_interface(bq24296_CON2, &ret_val, CON2_ICHG_MASK, CON2_ICHG_SHIFT);

    //Get Force 20% option
    bq24296_read_interface(bq24296_CON2, &ret_force_20pct, CON2_FORCE_20PCT_MASK, CON2_FORCE_20PCT_SHIFT);

    //Parsing
    ret_val = (ret_val*64) + 512;

    if (ret_force_20pct == 0) {
        //Get current level
        //array_size = GETARRAYNUM(CS_VTH);
        //*(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
        *(unsigned int *)data = ret_val;
    } else {
        //Get current level
        //array_size = GETARRAYNUM(CS_VTH_20PCT);
        //*(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
        //return (int)(ret_val<<1)/10;
        *(unsigned int *)data = (int)(ret_val<<1)/10;
    }
#endif

    //Get current level
    array_size = GETARRAYNUM(CS_VTH);
    sm5414_read_interface(SM5414_CHGCTRL2, &reg_value, SM5414_CHGCTRL2_FASTCHG_MASK, SM5414_CHGCTRL2_FASTCHG_SHIFT);//FASTCHG
    *(unsigned int *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);

    return status;
}

static unsigned int charging_set_current(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int set_chr_current;
    unsigned int array_size;
    unsigned int register_value;
    unsigned int current_value = *(unsigned int *)data;

    array_size = GETARRAYNUM(CS_VTH);
    set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
    register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
    printk("[cgh][%s]register_value =%x\r\n",__func__,register_value);
    sm5414_set_fastchg(register_value);//FASTCHG

    return status;
}

static unsigned int charging_set_input_current(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int current_value = *(unsigned int *)data;
    unsigned int set_chr_current;
    unsigned int array_size;
    unsigned int register_value;

    array_size = GETARRAYNUM(INPUT_CS_VTH);
    set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, current_value);
    register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);
    printk("[cgh][%s]register_value =%x\r\n",__func__,register_value);

    sm5414_set_vbuslimit(register_value);//VBUSLIMIT

    return status;
}

static unsigned int charging_get_charging_status(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned char val;
    unsigned int ret;

    ret = sm5414_get_topoff_status(&val);
    if (ret == 0) {
        return ret;
    }
   printk("[cgh][%s]val =%d\r\n",__func__,val);
    //Fullcharged status
    if (val == 0x1) {
        *(unsigned int *)data = KAL_TRUE;
    } else {
        *(unsigned int *)data = KAL_FALSE;
    }

    return status;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
    unsigned int status = STATUS_OK;

    return status;
}

static unsigned int charging_set_hv_threshold(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int set_hv_voltage;
    unsigned int array_size;
    unsigned short register_value;
    unsigned int voltage = *(unsigned int*)(data);

    array_size = GETARRAYNUM(VCDT_HV_VTH);
    set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
    register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
    upmu_set_rg_vcdt_hv_vth(register_value);

    return status;
}

static unsigned int charging_get_hv_status(void *data)
{
    unsigned int status = STATUS_OK;

    *(kal_bool*)(data) = upmu_get_rgs_vcdt_hv_det();

    return status;
}

static unsigned int charging_get_battery_status(void *data)
{
    unsigned int status = STATUS_OK;

	upmu_set_baton_tdet_en(1);
    upmu_set_rg_baton_en(1);
    *(kal_bool*)(data) = upmu_get_rgs_baton_undet();

    return status;
}

static unsigned int charging_get_charger_det_status(void *data)
{
    unsigned int status = STATUS_OK;

	*(kal_bool *)(data) = upmu_get_rgs_chrdet();

    return status;
}

kal_bool charging_type_detection_done(void)
{
    return charging_type_det_done;
}

extern int hw_otg_get_aca_type(void);
static unsigned int charging_get_charger_type(void *data)
{
    unsigned int status = STATUS_OK;
    //add by zym for Y-cable
    int i_ACA_type = 0;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
    *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
    *(CHARGER_TYPE*)(data) = hw_charging_get_charger_type();
#endif

    //add by liu for Y-cable
    i_ACA_type = hw_otg_get_aca_type();
    printk("ACA_type =%d\r\n",i_ACA_type);
    if (0x4 == i_ACA_type)
        *(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
    //end by liu
    return status;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
    unsigned int status = STATUS_OK;
#if 0
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
    *(kal_bool*)(data) = KAL_FALSE;
#else
    if (slp_get_wake_reason() == WR_PCM_TIMER) {
        *(kal_bool*)(data) = KAL_TRUE;
    } else {
        *(kal_bool*)(data) = KAL_FALSE;
    }
    printk("slp_get_wake_reason=%d\n", slp_get_wake_reason());
#endif
#endif
    return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
    unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
    printk("charging_set_platform_reset\n");
    kernel_restart("battery service reboot system");
    //arch_reset(0,NULL);
#endif
    return status;
}

extern unsigned int get_boot_mode(void);
static unsigned int charging_get_platform_boot_mode(void *data)
{
    unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
    *(unsigned int*)(data) = get_boot_mode();

    printk("get_boot_mode=%d\n", get_boot_mode());
#endif
    return status;
}

static unsigned int charging_set_power_off(void *data)
{
    unsigned int status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
    printk("charging_set_power_off\n");
    //mt_power_off();
    kernel_power_off();
#endif

    return status;
}

static unsigned int charging_get_power_source(void *data)
{
    unsigned int status = STATUS_OK;

#if 0   //#if defined(MTK_POWER_EXT_DETECT)
    if (MT_BOARD_PHONE == mt_get_board_type()) {
        *(kal_bool *)data = KAL_FALSE;
    } else {
        *(kal_bool *)data = KAL_TRUE;
    }
#else
    *(kal_bool *)data = KAL_FALSE;
#endif

    return status;
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
    return STATUS_UNSUPPORTED;
}

static unsigned int charging_set_ta_current_pattern(void *data)
{
#ifdef CONFIG_MTK_SM5414_SUPPORT
	battery_log(BAT_LOG_FULL, "Not Support PE TA");
#else
    unsigned int increase = *(unsigned int*)(data);
    unsigned int charging_status = KAL_FALSE;

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
    //BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_340000_V;
    BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_350000_V;
#else
    BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_200000_V;
#endif

    charging_get_charging_status(&charging_status);
    if (KAL_FALSE == charging_status) {
        charging_set_cv_voltage(&cv_voltage);  //Set CV
        sm5414_set_vbuslimit(VBUSLIMIT_500mA);//VBUSLIMIT
#if 0
        mt_set_gpio_mode(GPIO_SM5414_CHGEN_PIN,GPIO_MODE_GPIO);
        mt_set_gpio_dir(GPIO_SM5414_CHGEN_PIN,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_SM5414_CHGEN_PIN,GPIO_OUT_ZERO);

        silicon_sm5414_set_gpio(false);
        //pinctrl_select_state(g_sm5414_dev.pinctrl,g_sm5414_dev.sm5414_charge_disable);
#endif
        sm5414_set_chgen(CHARGE_DIS);
    }

    if (increase == KAL_TRUE) {
        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 1");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 1");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 2");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 2");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 3");
        msleep(281);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 3");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 4");
        msleep(281);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 4");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 5");
        msleep(281);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 5");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() on 6");
        msleep(485);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_increase() off 6");
        msleep(50);

        battery_log(BAT_LOG_CRTI, "mtk_ta_increase() end \n");

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        msleep(200);
    } else {
        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 1");
        msleep(281);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 1");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 2");
        msleep(281);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 2");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 3");
        msleep(281);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 3");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 4");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 4");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 5");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 5");
        msleep(85);

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() on 6");
        msleep(485);

        sm5414_set_vbuslimit(VBUSLIMIT_100mA); /* 100mA */
        battery_log(BAT_LOG_FULL, "mtk_ta_decrease() off 6");
        msleep(50);

        battery_log(BAT_LOG_CRTI, "mtk_ta_decrease() end \n");

        sm5414_set_vbuslimit(VBUSLIMIT_500mA); /* 500mA */
    }
#endif
    return STATUS_OK;
}

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
void set_vusb_auxadc_irq(bool enable, bool flag)
{
    hrtimer_cancel(&diso_kthread_timer);

    DISO_Polling.reset_polling = KAL_TRUE;
    DISO_Polling.vusb_polling_measure.notify_irq_en = enable;
    DISO_Polling.vusb_polling_measure.notify_irq = flag;

    hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)), HRTIMER_MODE_REL);

    printk(" [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}

void set_vdc_auxadc_irq(bool enable, bool flag)
{
    hrtimer_cancel(&diso_kthread_timer);

    DISO_Polling.reset_polling = KAL_TRUE;
    DISO_Polling.vdc_polling_measure.notify_irq_en = enable;
    DISO_Polling.vdc_polling_measure.notify_irq = flag;

    hrtimer_start(&diso_kthread_timer, ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)), HRTIMER_MODE_REL);
    printk(" [%s] enable: %d, flag: %d!\n", __func__, enable, flag);
}

static void diso_polling_handler(struct work_struct *work)
{
    int trigger_channel = -1;
    int trigger_flag = -1;

    if (DISO_Polling.vdc_polling_measure.notify_irq_en) {
        trigger_channel = AP_AUXADC_DISO_VDC_CHANNEL;
    } else if (DISO_Polling.vusb_polling_measure.notify_irq_en) {
        trigger_channel = AP_AUXADC_DISO_VUSB_CHANNEL;
    }
    printk("[DISO]auxadc handler triggered\n" );

    switch(trigger_channel) {
    case AP_AUXADC_DISO_VDC_CHANNEL:
        trigger_flag = DISO_Polling.vdc_polling_measure.notify_irq;
        printk("[DISO]VDC IRQ triggered, channel ==%d, flag ==%d\n", trigger_channel, trigger_flag );
#ifdef MTK_DISCRETE_SWITCH /*for DSC DC plugin handle */
        set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
        set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
        set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_FALLING);
        if (trigger_flag == DISO_IRQ_RISING) {
            DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
            DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
            DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
            DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
            DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
            DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
            printk(" cur diso_state is %s!\n",DISO_state_s[2]);
        }
#else //for load switch OTG leakage handle
        set_vdc_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag) & 0x1);
        if (trigger_flag == DISO_IRQ_RISING) {
            DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
            DISO_data.diso_state.pre_vdc_state  = DISO_OFFLINE;
            DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
            DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
            DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
            DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
            printk(" cur diso_state is %s!\n",DISO_state_s[5]);
        } else if (trigger_flag == DISO_IRQ_FALLING) {
            DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
            DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
            DISO_data.diso_state.pre_otg_state  = DISO_ONLINE;
            DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
            DISO_data.diso_state.cur_vdc_state  = DISO_OFFLINE;
            DISO_data.diso_state.cur_otg_state  = DISO_ONLINE;
            printk(" cur diso_state is %s!\n",DISO_state_s[1]);
        } else {
            printk("[%s] wrong trigger flag!\n",__func__);
        }
#endif
        break;
    case AP_AUXADC_DISO_VUSB_CHANNEL:
        trigger_flag = DISO_Polling.vusb_polling_measure.notify_irq;
        printk("[DISO]VUSB IRQ triggered, channel = %d, flag = %d\n", trigger_channel, trigger_flag);
        set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
        set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
        if (trigger_flag == DISO_IRQ_FALLING) {
            DISO_data.diso_state.pre_vusb_state  = DISO_ONLINE;
            DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
            DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
            DISO_data.diso_state.cur_vusb_state  = DISO_OFFLINE;
            DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
            DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
            printk(" cur diso_state is %s!\n",DISO_state_s[4]);
        } else if (trigger_flag == DISO_IRQ_RISING) {
            DISO_data.diso_state.pre_vusb_state  = DISO_OFFLINE;
            DISO_data.diso_state.pre_vdc_state  = DISO_ONLINE;
            DISO_data.diso_state.pre_otg_state  = DISO_OFFLINE;
            DISO_data.diso_state.cur_vusb_state  = DISO_ONLINE;
            DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
            DISO_data.diso_state.cur_otg_state  = DISO_OFFLINE;
            printk(" cur diso_state is %s!\n",DISO_state_s[6]);
        }
        else
            printk("[%s] wrong trigger flag!\n",__func__);
        set_vusb_auxadc_irq(DISO_IRQ_ENABLE, (~trigger_flag)&0x1);
        break;
    default:
        set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
        set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
        printk("[DISO]VUSB auxadc IRQ triggered ERROR OR TEST\n");
        return; /* in error or unexecpt state just return */
    }

    g_diso_state = *(int*)&DISO_data.diso_state;
    printk("[DISO]g_diso_state: 0x%x\n", g_diso_state);
    DISO_data.irq_callback_func(0, NULL);

    return ;
}

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
void vdc_eint_handler()
{
    printk("[diso_eint] vdc eint irq triger\n");
    DISO_data.diso_state.cur_vdc_state  = DISO_ONLINE;
    mt_eint_mask(CUST_EINT_VDC_NUM);
    do_chrdet_int_task();
}
#endif

static unsigned int diso_get_current_voltage(int Channel)
{
    int ret = 0, data[4], i, ret_value = 0, ret_temp = 0, times = 5;

    if ( IMM_IsAdcInitReady() == 0 ) {
        printk("[DISO] AUXADC is not ready");
        return 0;
    }

    i = times;
    while (i--) {
        ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);

        if (ret_value == 0) {
            ret += ret_temp;
        } else {
            times = times > 1 ? times - 1 : 1;
            printk("[diso_get_current_voltage] ret_value=%d, times=%d\n",
                ret_value, times);
        }
    }

    ret = ret*1500/4096 ;
    ret = ret/times;

    return  ret;
}

static void _get_diso_interrupt_state(void)
{
    int vol = 0;
    int diso_state = 0;
    int check_times = 30;
    kal_bool vin_state = KAL_FALSE;
#ifndef VIN_SEL_FLAG
    mdelay(AUXADC_CHANNEL_DELAY_PERIOD);
#endif

    vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
    vol =(R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN)*100*vol/(R_DISO_DC_PULL_DOWN)/100;
    printk("[DISO]  Current DC voltage mV = %d\n", vol);

#ifdef VIN_SEL_FLAG
    /* set gpio mode for kpoc issue as DWS has no default setting */
    mt_set_gpio_mode(vin_sel_gpio_number,0); // 0:GPIO mode
    mt_set_gpio_dir(vin_sel_gpio_number,0); // 0: input, 1: output

    if (vol > VDC_MIN_VOLTAGE/1000 && vol < VDC_MAX_VOLTAGE/1000) {
        /* make sure load switch already switch done */
        do {
            check_times--;
#ifdef VIN_SEL_FLAG_DEFAULT_LOW
            vin_state = mt_get_gpio_in(vin_sel_gpio_number);
#else
            vin_state = mt_get_gpio_in(vin_sel_gpio_number);
            vin_state = (~vin_state) & 0x1;
#endif
            if (!vin_state)
                mdelay(5);
        } while ((!vin_state) && check_times);
        printk("[DISO] i==%d  gpio_state= %d\n",
        check_times, mt_get_gpio_in(vin_sel_gpio_number));

        if (0 == check_times) {
            diso_state &= ~0x4; //SET DC bit as 0
        } else {
            diso_state |= 0x4; //SET DC bit as 1
        }
    } else {
        diso_state &= ~0x4; //SET DC bit as 0
    }
#else
    mdelay(SWITCH_RISING_TIMING + LOAD_SWITCH_TIMING_MARGIN); /* force delay for switching as no flag for check switching done */
    if (vol > VDC_MIN_VOLTAGE/1000 && vol < VDC_MAX_VOLTAGE/1000) {
        diso_state |= 0x4; //SET DC bit as 1
    } else {
        diso_state &= ~0x4; //SET DC bit as 0
    }
#endif

    vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
    vol = (R_DISO_VBUS_PULL_UP + R_DISO_VBUS_PULL_DOWN)*100*vol/(R_DISO_VBUS_PULL_DOWN)/100;
    printk("[DISO]  Current VBUS voltage  mV = %d\n",vol);

    if (vol > VBUS_MIN_VOLTAGE/1000 && vol < VBUS_MAX_VOLTAGE/1000) {
        if (!mt_usb_is_device()) {
            diso_state |= 0x1; //SET OTG bit as 1
            diso_state &= ~0x2; //SET VBUS bit as 0
        } else {
            diso_state &= ~0x1; //SET OTG bit as 0
            diso_state |= 0x2; //SET VBUS bit as 1;
        }
    } else {
        diso_state &= 0x4; //SET OTG and VBUS bit as 0
    }
    printk("[DISO] DISO_STATE==0x%x \n",diso_state);
    g_diso_state = diso_state;
    return;
}

int _get_irq_direction(int pre_vol, int cur_vol)
{
    int ret = -1;

    //threshold 1000mv
    if ((cur_vol - pre_vol) > 1000) {
        ret = DISO_IRQ_RISING;
    } else if ((pre_vol - cur_vol) > 1000) {
        ret = DISO_IRQ_FALLING;
    } else {

    }
    return ret;
}

static void _get_polling_state(void)
{
    int vdc_vol = 0, vusb_vol = 0;
    int vdc_vol_dir = -1;
    int vusb_vol_dir = -1;

    DISO_polling_channel* VDC_Polling = &DISO_Polling.vdc_polling_measure;
    DISO_polling_channel* VUSB_Polling = &DISO_Polling.vusb_polling_measure;

    vdc_vol = diso_get_current_voltage(AP_AUXADC_DISO_VDC_CHANNEL);
    vdc_vol =(R_DISO_DC_PULL_UP + R_DISO_DC_PULL_DOWN)*100*vdc_vol/(R_DISO_DC_PULL_DOWN)/100;

    vusb_vol = diso_get_current_voltage(AP_AUXADC_DISO_VUSB_CHANNEL);
    vusb_vol =(R_DISO_VBUS_PULL_UP + R_DISO_VBUS_PULL_DOWN)*100*vusb_vol/(R_DISO_VBUS_PULL_DOWN)/100;

    VDC_Polling->preVoltage = VDC_Polling->curVoltage;
    VUSB_Polling->preVoltage = VUSB_Polling->curVoltage;
    VDC_Polling->curVoltage = vdc_vol;
    VUSB_Polling->curVoltage = vusb_vol;

    if (DISO_Polling.reset_polling) {
        DISO_Polling.reset_polling = KAL_FALSE;
        VDC_Polling->preVoltage = vdc_vol;
        VUSB_Polling->preVoltage = vusb_vol;

        if (vdc_vol > 1000) {
            vdc_vol_dir = DISO_IRQ_RISING;
        } else {
            vdc_vol_dir = DISO_IRQ_FALLING;
        }
        if (vusb_vol > 1000) {
            vusb_vol_dir = DISO_IRQ_RISING;
        } else {
            vusb_vol_dir = DISO_IRQ_FALLING;
        }
    } else {
        //get voltage direction
        vdc_vol_dir = _get_irq_direction(VDC_Polling->preVoltage, VDC_Polling->curVoltage);
        vusb_vol_dir = _get_irq_direction(VUSB_Polling->preVoltage, VUSB_Polling->curVoltage);
    }

    if (VDC_Polling->notify_irq_en &&
        (vdc_vol_dir == VDC_Polling->notify_irq)) {
        schedule_delayed_work(&diso_polling_work, 10*HZ/1000); //10ms
        printk("[%s] ready to trig VDC irq, irq: %d\n",
            __func__,VDC_Polling->notify_irq);
    } else if (VUSB_Polling->notify_irq_en &&
        (vusb_vol_dir == VUSB_Polling->notify_irq)) {
        schedule_delayed_work(&diso_polling_work, 10*HZ/1000);
        printk("[%s] ready to trig VUSB irq, irq: %d\n",
            __func__, VUSB_Polling->notify_irq);
    } else if ((vdc_vol == 0) && (vusb_vol == 0)) {
        VDC_Polling->notify_irq_en = 0;
        VUSB_Polling->notify_irq_en = 0;
    }

    return;
}

enum hrtimer_restart diso_kthread_hrtimer_func(struct hrtimer *timer)
{
    diso_thread_timeout = KAL_TRUE;
    wake_up(&diso_polling_thread_wq);

    return HRTIMER_NORESTART;
}

int diso_thread_kthread(void *x)
{
    /* Run on a process content */
    while (1) {
        wait_event(diso_polling_thread_wq, (diso_thread_timeout == KAL_TRUE));

        diso_thread_timeout = KAL_FALSE;

        mutex_lock(&diso_polling_mutex);

        _get_polling_state();

        if (DISO_Polling.vdc_polling_measure.notify_irq_en ||
            DISO_Polling.vusb_polling_measure.notify_irq_en) {
            hrtimer_start(&diso_kthread_timer,ktime_set(0, MSEC_TO_NSEC(SW_POLLING_PERIOD)),HRTIMER_MODE_REL);
        } else {
            hrtimer_cancel(&diso_kthread_timer);
        }
        mutex_unlock(&diso_polling_mutex);
    }

    return 0;
}
#endif

static unsigned int charging_diso_init(void *data)
{
    unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
    DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

    /* Initialization DISO Struct */
    pDISO_data->diso_state.cur_otg_state    = DISO_OFFLINE;
    pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
    pDISO_data->diso_state.cur_vdc_state    = DISO_OFFLINE;

    pDISO_data->diso_state.pre_otg_state    = DISO_OFFLINE;
    pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
    pDISO_data->diso_state.pre_vdc_state    = DISO_OFFLINE;

    pDISO_data->chr_get_diso_state = KAL_FALSE;
    pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

    hrtimer_init(&diso_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    diso_kthread_timer.function = diso_kthread_hrtimer_func;
    INIT_DELAYED_WORK(&diso_polling_work, diso_polling_handler);

    kthread_run(diso_thread_kthread, NULL, "diso_thread_kthread");
    printk("[%s] done\n", __func__);

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
    printk("[diso_eint]vdc eint irq registitation\n");
    mt_eint_set_hw_debounce(CUST_EINT_VDC_NUM, CUST_EINT_VDC_DEBOUNCE_CN);
    mt_eint_registration(CUST_EINT_VDC_NUM, CUST_EINTF_TRIGGER_LOW, vdc_eint_handler, 0);
    mt_eint_mask(CUST_EINT_VDC_NUM);
#endif
#endif

    return status;
}

static unsigned int charging_get_diso_state(void *data)
{
    unsigned int status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
    int diso_state = 0x0;
    DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

    _get_diso_interrupt_state();
    diso_state = g_diso_state;
    printk("[do_chrdet_int_task] current diso state is %s!\n", DISO_state_s[diso_state]);
    if (((diso_state >> 1) & 0x3) != 0x0) {
        switch (diso_state) {
        case USB_ONLY:
            set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
            set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
#ifdef MTK_DSC_USE_EINT
            mt_eint_unmask(CUST_EINT_VDC_NUM);
#else
            set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
#endif
            pDISO_data->diso_state.cur_vusb_state  = DISO_ONLINE;
            pDISO_data->diso_state.cur_vdc_state    = DISO_OFFLINE;
            pDISO_data->diso_state.cur_otg_state    = DISO_OFFLINE;
            break;
        case DC_ONLY:
            set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
            set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
            set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
            pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
            pDISO_data->diso_state.cur_vdc_state    = DISO_ONLINE;
            pDISO_data->diso_state.cur_otg_state    = DISO_OFFLINE;
            break;
        case DC_WITH_USB:
            set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
            set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
            set_vusb_auxadc_irq(DISO_IRQ_ENABLE,DISO_IRQ_FALLING);
            pDISO_data->diso_state.cur_vusb_state  = DISO_ONLINE;
            pDISO_data->diso_state.cur_vdc_state    = DISO_ONLINE;
            pDISO_data->diso_state.cur_otg_state    = DISO_OFFLINE;
            break;
        case DC_WITH_OTG:
            set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
            set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
            pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
            pDISO_data->diso_state.cur_vdc_state    = DISO_ONLINE;
            pDISO_data->diso_state.cur_otg_state    = DISO_ONLINE;
            break;
        default: // OTG only also can trigger vcdt IRQ
            pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
            pDISO_data->diso_state.cur_vdc_state    = DISO_OFFLINE;
            pDISO_data->diso_state.cur_otg_state    = DISO_ONLINE;
            printk(" switch load vcdt irq triggerd by OTG Boost!\n");
            break; // OTG plugin no need battery sync action
        }
    }

    if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state) {
        pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
    } else {
        pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;
    }
#endif

    return status;
}

static unsigned int charging_enable_otg(void *data)
{
    unsigned int status = STATUS_OK;
    unsigned int enable = *(unsigned int*)(data);

	printk("[charging_enable_otg]-enable(%d)-\n",enable);

	if (enable == KAL_TRUE)
		status = sm5414_otg_enable(ENBOOST_EN);
	else
		status = sm5414_otg_enable(ENBOOST_DIS);

	return status;
}

#if 0
static unsigned int charging_get_error_state(void)
{
    return charging_error;
}
#endif

static unsigned int charging_set_error_state(void *data)
{
    unsigned int status = STATUS_OK;
    charging_error = *(unsigned int*)(data);

    return status;
}

static unsigned int (* const charging_func[CHARGING_CMD_NUMBER])(void *data) = {
    charging_hw_init,
    charging_dump_register,
    charging_enable,
    charging_set_cv_voltage,
    charging_get_current,
    charging_set_current,
    charging_set_input_current,
    charging_get_charging_status,
    charging_reset_watch_dog_timer,
    charging_set_hv_threshold,
    charging_get_hv_status,
    charging_get_battery_status,
    charging_get_charger_det_status,
    charging_get_charger_type,
    charging_get_is_pcm_timer_trigger,
    charging_set_platform_reset,
    charging_get_platform_boot_mode,
    charging_set_power_off,
    charging_get_power_source,
    charging_get_csdac_full_flag,
    charging_set_ta_current_pattern,
    charging_set_error_state,
    charging_diso_init,
    charging_get_diso_state,
    charging_enable_otg
};

/*
* FUNCTION: Internal_chr_control_handler
* DESCRIPTION: This function is called to set the charger hw
* CALLS
* PARAMETERS: None
* RETURNS
* GLOBALS AFFECTED:  None
*/
int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
    int status;
    if ((cmd < CHARGING_CMD_NUMBER) && (charging_func[cmd])) {
        status = charging_func[cmd](data);
    } else {
        status = STATUS_UNSUPPORTED;
    }
    return status;
}
