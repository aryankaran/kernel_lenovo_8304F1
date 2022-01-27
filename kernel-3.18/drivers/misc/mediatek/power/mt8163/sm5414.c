#include <linux/types.h>
#include <linux/init.h>     /* For init/exit macros */
#include <linux/module.h>   /* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/pinctrl/consumer.h>
#include <mt-plat/charging.h>
//#include <mt-plat/mtk_gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>

#include <linux/delay.h>
#include "sm5414.h"


#define GPIO_SM5414_CHGEN_PIN(x)        (x | 0x80000000)
#define GPIO_SM5414_nSHDN_PIN(x)        (x | 0x80000000)
#define GPIO_SM5414_EINT_PIN(x)         (x | 0x80000000)

#define sm5414_SLAVE_ADDR_WRITE         0x92
#define sm5414_SLAVE_ADDR_READ          0x93

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0

#define sm5414_BUSNUM                   1

#define SM_INFO(fmt,arg...)             printk(KERN_INFO "[CHG_SM5414]%s "fmt,__func__,##arg);
#define SM_ERR(fmt,arg...)              printk(KERN_ERR "[CHG_SM5414]%s "fmt,__func__,##arg);

static struct i2c_client *new_client = NULL;

kal_bool chargin_hw_init_done = KAL_FALSE;

struct sm5414_info {
    struct device *pdev;        /* the device structure */
    struct i2c_client *i2c;     /* i2c */
	struct work_struct 	pen_event_work;
	struct workqueue_struct *otg_check_workqueue;
    int nCHGEN;                 /* gpio17 */
    int nINT;                   /* gpio4 */
    int nSHDN;                  /* gpio12 */
    int irq;                    /* interrupt */
    int otg_cnt;                /* otg count */
};

unsigned char sm5414_reg[SM5414_REG_NUM] = {0};

static DEFINE_MUTEX(sm5414_i2c_access);

int g_sm5414_hw_exist = 0;

//I2C Function For Read/Write sm5414
static int sm5414_read_byte(unsigned char reg, unsigned char *data)
{
    int ret = -1;

    struct i2c_msg msg[2] = {
        {
            .addr = new_client->addr,
            .buf = &reg,
            .flags = 0,
            .len = 1,
        },
        {
            .addr = new_client->addr,
            .buf = data,
            .flags = I2C_M_RD,
            .len = 1,
        }
    };
    mutex_lock(&sm5414_i2c_access);

    ret = i2c_transfer(new_client->adapter, msg, 2);
    if (ret < 0) {
        SM_ERR("faild(%d)\n",ret);
        mutex_unlock(&sm5414_i2c_access);
        return 0;
    }
    mutex_unlock(&sm5414_i2c_access);
    return 1;
}

static int sm5414_write_byte(unsigned char reg, unsigned char data)
{
    char buf[2] = {0};
    int ret = 0;
    struct i2c_msg msg = {
        .addr = new_client->addr,
        .flags = 0,
        .buf = buf,
        .len = 2,
    };

    buf[0] = reg;
    buf[1] = data;
    mutex_lock(&sm5414_i2c_access);

    ret = i2c_transfer(new_client->adapter, &msg, 1);
    if (ret < 0) {
        mutex_unlock(&sm5414_i2c_access);
        SM_ERR("faild(%d)\n",ret);
        return 0;
    }
    mutex_unlock(&sm5414_i2c_access);
    return 1;
}

unsigned int sm5414_read_interface (unsigned char RegNum, unsigned char *val,
                unsigned char MASK, unsigned char SHIFT)
{
    unsigned char sm5414_reg = 0;
    int ret = 0;

    ret = sm5414_read_byte(RegNum, &sm5414_reg);
    if (ret == 0) {
        return ret;
    }
    SM_INFO("Reg[%x] = 0x%x\n", RegNum, sm5414_reg);

    sm5414_reg &= (MASK << SHIFT);
    *val = (sm5414_reg >> SHIFT);

    SM_INFO("val = 0x%x\n", *val);

    return ret;
}

unsigned int sm5414_config_interface (unsigned char RegNum, unsigned char val,
                unsigned char MASK, unsigned char SHIFT)
{
    unsigned char sm5414_reg = 0;
    int ret = 0;

    ret = sm5414_read_byte(RegNum, &sm5414_reg);
    if (ret == 0) {
        return ret;
    }

    SM_INFO("Reg[0x%x] = 0x%x\n", RegNum, sm5414_reg);

    sm5414_reg &= ~(MASK << SHIFT);
    sm5414_reg |= (val << SHIFT);

    ret = sm5414_write_byte(RegNum, sm5414_reg);
    if (ret == 0) {
        return ret;
    }

    SM_INFO("write Reg[0x%x] = 0x%x\n", RegNum, sm5414_reg);

    return ret;
}

//write one register directly
unsigned int sm5414_reg_config_interface (unsigned char RegNum, unsigned char val)
{
    return sm5414_write_byte(RegNum, val);
}

unsigned int sm5414_get_topoff_status(unsigned char *val)
{
    return sm5414_read_interface((unsigned char)(SM5414_STATUS),val,
                                (unsigned char)(SM5414_STATUS_TOPOFF_MASK),
                                (unsigned char)(SM5414_STATUS_TOPOFF_SHIFT));
}

void sm5414_set_enboost(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_ENBOOST_MASK),
                            (unsigned char)(SM5414_CTRL_ENBOOST_SHIFT));
}

void sm5414_set_chgen(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_CHGEN_MASK),
                            (unsigned char)(SM5414_CTRL_CHGEN_SHIFT));
}

void sm5414_set_suspen(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_SUSPEN_MASK),
                            (unsigned char)(SM5414_CTRL_SUSPEN_SHIFT));
}

void sm5414_set_reset(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_RESET_MASK),
                            (unsigned char)(SM5414_CTRL_RESET_SHIFT));
}

void sm5414_set_encomparator(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_CTRL_ENCOMPARATOR_MASK),
                            (unsigned char)(SM5414_CTRL_ENCOMPARATOR_SHIFT));
}

//vbusctrl
void sm5414_set_vbuslimit(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_VBUSCTRL),(unsigned char)(val),
                            (unsigned char)(SM5414_VBUSCTRL_VBUSLIMIT_MASK),
                            (unsigned char)(SM5414_VBUSCTRL_VBUSLIMIT_SHIFT));
}

//chgctrl1
void sm5414_set_prechg(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL1),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL1_PRECHG_MASK),
                            (unsigned char)(SM5414_CHGCTRL1_PRECHG_SHIFT));
}

void sm5414_set_aiclen(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL1),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL1_AICLEN_MASK),
                            (unsigned char)(SM5414_CHGCTRL1_AICLEN_SHIFT));
}

void sm5414_set_autostop(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL1),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL1_AUTOSTOP_MASK),
                            (unsigned char)(SM5414_CHGCTRL1_AUTOSTOP_SHIFT));
}
void sm5414_set_aiclth(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL1),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL1_AICLTH_MASK),
                            (unsigned char)(SM5414_CHGCTRL1_AICLTH_SHIFT));
}

//chgctrl2
void sm5414_set_fastchg(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL2),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL2_FASTCHG_MASK),
                            (unsigned char)(SM5414_CHGCTRL2_FASTCHG_SHIFT));
}

//chgctrl3
void sm5414_set_weakbat(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL3),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL3_WEAKBAT_MASK),
                            (unsigned char)(SM5414_CHGCTRL3_WEAKBAT_SHIFT));
}
void sm5414_set_batreg(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL3),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL3_BATREG_MASK),
                            (unsigned char)(SM5414_CHGCTRL3_BATREG_SHIFT));
}

//chgctrl4
void sm5414_set_dislimit(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL4),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL4_DISLIMIT_MASK),
                            (unsigned char)(SM5414_CHGCTRL4_DISLIMIT_SHIFT));
}

void sm5414_set_topoff(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL4),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL4_TOPOFF_MASK),
                            (unsigned char)(SM5414_CHGCTRL4_TOPOFF_SHIFT));
}

//chgctrl5
void sm5414_set_topofftimer(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL5),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL5_TOPOFFTIMER_MASK),
                            (unsigned char)(SM5414_CHGCTRL5_TOPOFFTIMER_SHIFT));
}

void sm5414_set_fasttimer(unsigned int val)
{

    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL5),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL5_FASTTIMER_MASK),
                            (unsigned char)(SM5414_CHGCTRL5_FASTTIMER_SHIFT));
}

void sm5414_set_votg(unsigned int val)
{
    sm5414_config_interface((unsigned char)(SM5414_CHGCTRL5),(unsigned char)(val),
                            (unsigned char)(SM5414_CHGCTRL5_VOTG_MASK),
                            (unsigned char)(SM5414_CHGCTRL5_VOTG_SHIFT));
}

int sm5414_chg_enable_gpio(unsigned int enable)
{
    struct sm5414_info *info = i2c_get_clientdata(new_client);

    if (info == NULL) {
        return -EINVAL;
    }
    if (KAL_TRUE == enable) {
        gpio_direction_output(info->nCHGEN, GPIO_OUT_ZERO);
    } else {
        gpio_direction_output(info->nCHGEN, GPIO_OUT_ONE);
    }
    return 0;
}

int sm5414_otg_enable(unsigned int enable)
{
    struct sm5414_info *info = i2c_get_clientdata(new_client);

    if (info == NULL) {
        return -EINVAL;
    }

    if (KAL_TRUE == enable) {
        //Before turning on OTG, system must turn off charing function.
        sm5414_set_chgen(CHARGE_DIS);
        sm5414_set_enboost(ENBOOST_EN);
        info->otg_cnt = 0;
    } else {
		sm5414_set_suspen(SUSPEND_EN);	/*add by zhengxianjin for disable power path 20170420*/
		mdelay(1);
        sm5414_set_enboost(ENBOOST_DIS);
		mdelay(1);
		sm5414_set_suspen(SUSPEND_DIS);	/*add by zhengxianjin for  enable power path 20170420*/
    }

    return 0;
}

int is_sm5414_exist(void)
{
    SM_INFO("g_sm5414_hw_exist = %d\n", g_sm5414_hw_exist);

    return g_sm5414_hw_exist;
}

void sm5414_dump_register(void)
{
    int i = 0;
    printk("dump regs\n");
    for (i = SM5414_INTMASK1; i < SM5414_REG_NUM; i++) {
        sm5414_read_byte(i, &sm5414_reg[i]);
        printk("[0x%x]=0x%x ", i, sm5414_reg[i]);
    }
    printk("\n");
}

unsigned int sm5414_hw_component_detect(void)
{
    unsigned char val = 0;
    int ret = 0;

    ret = sm5414_read_interface(0x0E, &val, 0xFF, 0x0);
    if (ret == 0) {
        return ret;
    }

    g_sm5414_hw_exist = 1;

    SM_INFO("exist = %d, Reg[0x0E] = 0x%x\n", g_sm5414_hw_exist, val);
    return g_sm5414_hw_exist;
}

void sm5414_reg_init(void)
{
    //INT MASK 1/2/3
    sm5414_write_byte(SM5414_INTMASK1, 0xFF);
    sm5414_write_byte(SM5414_INTMASK2, 0xEF); //OTGFAILM
    sm5414_write_byte(SM5414_INTMASK3, 0xFF);

    sm5414_set_encomparator(ENCOMPARATOR_EN);
    sm5414_set_topoff(TOPOFF_100mA);
    sm5414_set_batreg(BATREG_4_3_7_5_V);

#if defined(SM5414_TOPOFF_TIMER_SUPPORT)
    sm5414_set_autostop(AUTOSTOP_EN);
    sm5414_set_topofftimer(TOPOFFTIMER_10MIN);
#else
    sm5414_set_autostop(AUTOSTOP_DIS);
#endif
}

static void sm5414_otg_worker(struct work_struct *work)
{
	struct sm5414_info *info = i2c_get_clientdata(new_client);
     u8 int_value[3] = {0};

     sm5414_read_byte(SM5414_INT1, &int_value[SM5414_INT1]);
     sm5414_read_byte(SM5414_INT2, &int_value[SM5414_INT2]);
     sm5414_read_byte(SM5414_INT3, &int_value[SM5414_INT3]);

     SM_INFO("INT1 : 0x%x, INT2 : 0x%x, INT3 : 0x%x\n", int_value[SM5414_INT1],int_value[SM5414_INT2],int_value[SM5414_INT3]);

     if (int_value[SM5414_INT2] & SM5414_INT2_OTGFAIL) {
         SM_INFO("OTG FAIL is occurred!!\n");
         //When OTG boost Fail is occurred, OTG boost is retried one more time.
         sm5414_otg_enable(ENBOOST_DIS);
         if (info->otg_cnt < 1) {
            sm5414_otg_enable(ENBOOST_EN);
            msleep(80);
            info->otg_cnt++;
         }
     }
}

static irqreturn_t sm5414_irq_handler(int irq, void *dev_id)
{
	struct sm5414_info *info = (struct sm5414_info *)dev_id;
	disable_irq_nosync(info->irq);
	if (!work_pending(&info->pen_event_work)) {
		queue_work(info->otg_check_workqueue, &info->pen_event_work);
	}

     return IRQ_HANDLED;
}

static int sm5414_irq_init(struct sm5414_info *info)
{
     int ret = 0;
     SM_INFO("Start\n");

     if (info->pdev->of_node) {
		info->irq = irq_of_parse_and_map(info->pdev->of_node, 0);
        SM_INFO("info->irq = %d\n", info->irq);

        if (info->irq <= 0) {
            SM_ERR("request_irq IRQ LINE NOT AVAILABLE!.\n");
            return -1;
        }
        ret = request_threaded_irq(info->irq, NULL, (irq_handler_t)sm5414_irq_handler,
                    IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT, "sm5414-irq", info);
        if (ret < 0) {
            SM_ERR("request_irq IRQ LINE NOT AVAILABLE!.\n");
        }
     } else {
         SM_INFO("request_irq can not find  eint device node!.\n");
         ret = -1;
     }

     SM_INFO("Done\n");

     return ret;
}

static int sm5414_gpio_init(struct sm5414_info *info)
{
	if(gpio_request_one(info->nCHGEN, GPIOF_OUT_INIT_LOW, "GPIO_BAT_CHGEN") < 0) {
		SM_ERR("gpio %d request failed.\n", info->nCHGEN);
		gpio_free(info->nCHGEN);
		return -1;
	}
    mdelay(10);		//delay is neccesary

	if(gpio_request_one(info->nSHDN, GPIOF_OUT_INIT_HIGH, "GPIO_BAT_SHDN") < 0) {
		SM_ERR("gpio %d request failed.\n", info->nSHDN);
		gpio_free(info->nSHDN);
		return -1;
	}
    mdelay(10);		//delay is neccesary

	if(gpio_request_one(info->nINT, GPIOF_IN, "CHRG_IN") < 0) {
		SM_ERR("gpio %d request failed.\n", info->nINT);
		gpio_free(info->nINT);
		return -1;
	}
    mdelay(10);		//delay is neccesary
	return 0;
}

static int sm5414_parse_dt(struct device *dev, struct sm5414_info *info)
{
     struct device_node *np = dev->of_node;

     info->nCHGEN = of_get_named_gpio(np, "gpio_pwr_chgen" ,0);
     info->nINT = of_get_named_gpio(np, "gpio_chrg_nint" ,0);
     info->nSHDN = of_get_named_gpio(np, "gpio_chrg_shdn" ,0);

     return 0;
}

static int sm5414_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct sm5414_info *info = NULL;

    SM_INFO("start\n");

    info = (struct sm5414_info*)kzalloc(sizeof(*info), GFP_KERNEL);
    if (!info) {
        SM_ERR("sm5414_info memory allocation failed.\n");
        return -ENOMEM;
    }

    /* initialize device info */
    info->i2c = client;
    info->pdev = &client->dev;
    info->otg_cnt = 0;

    new_client = client;

    /* read device tree */
    if (client->dev.of_node) {
        ret = sm5414_parse_dt(&client->dev, info);
        if (ret) {
            SM_ERR("cannot read from dt.\n");
            goto err;
        }
    }

    /* initialize device */
   if( sm5414_gpio_init(info) < 0) {
		SM_ERR("cannot init gpio for chrg.\n");
		goto err;
   }

    i2c_set_clientdata(client, info);

    ret = sm5414_hw_component_detect();
    if (ret == 0) {
        goto err;
    }

	INIT_WORK(&info->pen_event_work, sm5414_otg_worker);
	info->otg_check_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!info->otg_check_workqueue) {
		goto exit_init_workqueue;
	}

    sm5414_reg_init();

    ret = sm5414_irq_init(info);
    if (ret < 0) {
        SM_ERR("Error : can't initialize SM5414 irq.\n");
        goto exit_init_workqueue;
    }

    chargin_hw_init_done = KAL_TRUE;

    sm5414_dump_register();

    SM_INFO("DONE\n");

    return 0;

exit_init_workqueue:
	cancel_work_sync(&info->pen_event_work);
	destroy_workqueue(info->otg_check_workqueue);
err:
	kfree(info);
    return -1;
}

/*add by zhengxianjin for nSHDN low*/
static int sm5414_driver_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm5414_info *info = i2c_get_clientdata(client);
	SM_INFO("SUSPEND DEBUG");
	if(info == NULL)
	  return -EINVAL;
	gpio_direction_output(info->nSHDN, GPIO_OUT_ZERO);
	disable_irq(info->irq);
	return 0;
}

static int sm5414_driver_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm5414_info *info = i2c_get_clientdata(client);
	SM_INFO("RESUME DEBUG");
	if(info == NULL)
	  return -EINVAL;
	gpio_direction_output(info->nSHDN, GPIO_OUT_ONE);
	enable_irq(info->irq);
	return 0;
}
/*add by zhengxianjin for nSHDN low*/

static unsigned char g_reg_value_sm5414 = 0;
static ssize_t show_sm5414_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    SM_INFO("0x%x\n", g_reg_value_sm5414);
    return sprintf(buf, "%u\n", g_reg_value_sm5414);
}

static ssize_t store_sm5414_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;

    SM_INFO("start\n");

    if ((buf != NULL) && (size != 0)) {
        SM_INFO("buf is %s and size is %zu\n",buf,size);
        reg_address = simple_strtoul(buf,&pvalue,16);

        if (size > 3) {
            reg_value = simple_strtoul((pvalue + 1),NULL,16);
            SM_INFO("write sm5414 reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            sm5414_config_interface(reg_address, reg_value, 0xFF, 0x0);
        } else {
            sm5414_read_interface(reg_address, &g_reg_value_sm5414, 0xFF, 0x0);
            SM_INFO("read sm5414 reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_sm5414);
            SM_INFO("Please use \"cat sm5414_access\" to get value\n");
        }
    }
    return size;
}

static DEVICE_ATTR(sm5414_access, 0664, show_sm5414_access, store_sm5414_access);

static int sm5414_user_space_probe(struct platform_device *dev)
{
    SM_INFO("entry\n" );

    return device_create_file(&(dev->dev), &dev_attr_sm5414_access);
}

struct platform_device sm5414_user_space_device = {
    .name = "sm5414-user",
    .id = -1,
};

static struct platform_driver sm5414_user_space_driver = {
    .probe = sm5414_user_space_probe,
    .driver = {
        .name = "sm5414-user",
    },
};

static const struct i2c_device_id sm5414_i2c_id[] = {
    {"sm5414",0},
    {}
};

#ifdef CONFIG_OF
static const struct of_device_id sm5414_of_match[] = {
    { .compatible = "sm,sm5414", },
    {},
};

MODULE_DEVICE_TABLE(of, sm5414_of_match);
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops sm5414_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sm5414_driver_suspend, sm5414_driver_resume)
};
#endif
static struct i2c_driver sm5414_driver = {
    .driver = {
        .name    = "sm5414",
#ifdef CONFIG_OF
        .of_match_table = sm5414_of_match,
#endif
#ifdef CONFIG_PM_SLEEP
		.pm		= &sm5414_pm_ops,
#endif
    },
    .probe       = sm5414_driver_probe,
    .id_table    = sm5414_i2c_id,
};

static int __init sm5414_init(void)
{
    int ret = 0;

    SM_INFO("start. ch=%d\n", sm5414_BUSNUM);

    ret = i2c_add_driver(&sm5414_driver);
    if (ret) {
        SM_ERR("failed to register sm5414 i2c driver.\n");
        goto err;
    } else {
        SM_INFO("Success to register sm5414 i2c driver.\n");
    }

    //sm5414 user space access interface
    ret = platform_device_register(&sm5414_user_space_device);
    if (ret) {
        SM_ERR("Unable to device register sm5414_user_space_device(%d)\n", ret);
        goto err2;
    }
    ret = platform_driver_register(&sm5414_user_space_driver);
    if (ret) {
        SM_ERR("Unable to register driver sm5414_user_space_driver(%d)\n", ret);
        goto err3;
    }

    return ret;

err3:
    platform_device_unregister(&sm5414_user_space_device);
err2:
    i2c_del_driver(&sm5414_driver);
err:
    return ret;
}

static void __exit sm5414_exit(void)
{
    sm5414_otg_enable(ENBOOST_DIS);

    platform_driver_unregister(&sm5414_user_space_driver);
    platform_device_unregister(&sm5414_user_space_device);
    i2c_del_driver(&sm5414_driver);
}

module_init(sm5414_init);
module_exit(sm5414_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C sm5414 Driver");
