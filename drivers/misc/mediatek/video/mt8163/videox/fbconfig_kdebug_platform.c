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

#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <linux/types.h>

#include "mtkfb.h"
#include "debug.h"
#include "lcm_drv.h"
#include "ddp_path.h"
#include "fbconfig_kdebug_platform.h"
#include "primary_display.h"
#include "ddp_ovl.h"
#include "ddp_dsi.h"

/* #include "disp_drv.h" */
/* #include "lcd_drv.h" */

/* **************************************************************************** */
/* This part is for customization parameters of D-IC and DSI . */
/* **************************************************************************** */


/* extern FBCONFIG_DISP_IF * fbconfig_if_drv; */
bool fbconfig_start_LCM_config;
#define FBCONFIG_MDELAY(n)	(PM_lcm_utils_dsi0.mdelay((n)))
#define SET_RESET_PIN(v)	(PM_lcm_utils_dsi0.set_reset_pin((v)))
#define dsi_set_cmdq(pdata, queue_size, force_update)	PM_lcm_utils_dsi0.dsi_set_cmdq(pdata, queue_size, force_update)
/* #define read_reg_v2(cmd, buffer, buffer_size)                         PM_lcm_utils_dsi0.dsi_dcs_read_lcm_reg_v2 */
#define FBCONFIG_KEEP_NEW_SETTING 1
#define FBCONFIG_DEBUG 0
/* IOCTL commands. */

#define FBCONFIG_IOW(num, dtype)     _IOW('X', num, dtype)
#define FBCONFIG_IOR(num, dtype)     _IOR('X', num, dtype)
#define FBCONFIG_IOWR(num, dtype)    _IOWR('X', num, dtype)
#define FBCONFIG_IO(num)             _IO('X', num)

#define GET_DSI_ID	   FBCONFIG_IOW(43, unsigned int)
#define SET_DSI_ID	   FBCONFIG_IOW(44, unsigned int)
#define LCM_GET_ID     FBCONFIG_IOR(45, unsigned int)
#define LCM_GET_ESD    FBCONFIG_IOWR(46, unsigned int)
#define DRIVER_IC_CONFIG    FBCONFIG_IOR(47, unsigned int)
#define DRIVER_IC_CONFIG_DONE  FBCONFIG_IO(0)
#define DRIVER_IC_RESET    FBCONFIG_IOR(48, unsigned int)


#define MIPI_SET_CLK     FBCONFIG_IOW(51, unsigned int)
#define MIPI_SET_LANE    FBCONFIG_IOW(52, unsigned int)
#define MIPI_SET_TIMING  FBCONFIG_IOW(53, unsigned int)
#define MIPI_SET_VM      FBCONFIG_IOW(54, unsigned int)	/* mipi video mode timing setting */
#define MIPI_SET_CC	 FBCONFIG_IOW(55, unsigned int)	/* mipi non-continuous clock */
#define MIPI_SET_SSC	 FBCONFIG_IOW(56, unsigned int)	/* spread frequency */
#define MIPI_SET_CLK_V2  FBCONFIG_IOW(57, unsigned int)	/* For div1,div2,fbk_div case */


#define TE_SET_ENABLE  FBCONFIG_IOW(61, unsigned int)
#define FB_LAYER_DUMP  FBCONFIG_IOW(62, unsigned int)
#define FB_LAYER_GET_INFO FBCONFIG_IOW(63, unsigned int)
#define FB_LAYER_GET_EN FBCONFIG_IOW(64, unsigned int)
#define LCM_GET_ESD_RET    FBCONFIG_IOR(65, unsigned int)

#define LCM_GET_DSI_CONTINU    FBCONFIG_IOR(71, unsigned int)
#define LCM_GET_DSI_CLK   FBCONFIG_IOR(72, unsigned int)
#define LCM_GET_DSI_TIMING   FBCONFIG_IOR(73, unsigned int)
#define LCM_GET_DSI_LANE_NUM    FBCONFIG_IOR(74, unsigned int)
#define LCM_GET_DSI_TE    FBCONFIG_IOR(75, unsigned int)
#define LCM_GET_DSI_SSC    FBCONFIG_IOR(76, unsigned int)
#define LCM_GET_DSI_CLK_V2    FBCONFIG_IOR(77, unsigned int)
#define LCM_TEST_DSI_CLK    FBCONFIG_IOR(78, unsigned int)

#define DP_COLOR_BITS_PER_PIXEL(color)    ((0x0003FF00 & color) >>  8)

struct dentry *ConfigPara_dbgfs = NULL;
CONFIG_RECORD_LIST head_list;
LCM_REG_READ reg_read;
/* int esd_check_addr; */
/* int esd_check_para_num; */
/* int esd_check_type; */
/* char * esd_check_buffer =NULL; */
/* extern void fbconfig_disp_set_mipi_timing(MIPI_TIMING timing); */
/* extern unsigned int fbconfig_get_layer_info(FBCONFIG_LAYER_INFO *layers); */
/* extern unsigned int fbconfig_get_layer_vaddr(int layer_id,int * layer_size,int * enable); */
/* unsigned int fbconfig_get_layer_height(int layer_id,int * layer_size,int * enable,int* height ,int * fmt); */

typedef struct PM_TOOL_ST {
	DSI_INDEX dsi_id;
	LCM_REG_READ reg_read;
	LCM_PARAMS *pLcm_params;
	LCM_DRIVER *pLcm_drv;
} PM_TOOL_T;
static PM_TOOL_T pm_params = {
	.dsi_id = PM_DSI0,
	.pLcm_params = NULL,
	.pLcm_drv = NULL,
};
struct mutex fb_config_lock;

static void *pm_get_handle(void)
{
	return (void *)&pm_params;
}

static DISP_MODULE_ENUM pm_get_dsi_handle(DSI_INDEX dsi_id)
{
	if (dsi_id == PM_DSI0)
		return DISP_MODULE_DSI0;
	else if (dsi_id == PM_DSI1)
		return DISP_MODULE_DSI1;
	else if (dsi_id == PM_DSI_DUAL)
		return DISP_MODULE_DSIDUAL;
	else
		return DISP_MODULE_UNKNOWN;
}

int fbconfig_get_esd_check(DSI_INDEX dsi_id, uint32_t cmd, uint8_t *buffer, uint32_t num)
{
	int array[4];
	int ret;
	/* set max return packet size */
	/* array[0] = 0x00013700; */
	array[0] = 0x3700 + (num << 16);
	dsi_set_cmdq(array, 1, 1);
	atomic_set(&ESDCheck_byCPU, 1);
	ret = DSI_dcs_read_lcm_reg_v2(pm_get_dsi_handle(dsi_id), NULL, cmd, buffer, num);
	atomic_set(&ESDCheck_byCPU, 0);
	if (ret == 0)
		return -1;

	return 0;
}

/* RECORD_CMD = 0, */
/* RECORD_MS = 1, */
/* RECORD_PIN_SET        = 2, */

void Panel_Master_DDIC_config(void)
{

	struct list_head *p;
	CONFIG_RECORD_LIST *node;

	list_for_each_prev(p, &head_list.list) {
		node = list_entry(p, CONFIG_RECORD_LIST, list);
		switch (node->record.type) {
		case RECORD_CMD:
			dsi_set_cmdq(node->record.ins_array, node->record.ins_num, 1);
			break;
		case RECORD_MS:
			FBCONFIG_MDELAY(node->record.ins_array[0]);
			break;
		case RECORD_PIN_SET:
			SET_RESET_PIN(node->record.ins_array[0]);
			break;
		default:
			pr_debug("sxk=>No such Type!!!!!\n");
		}

	}

}

#if 0
static void print_from_head_to_tail(void)
{
	int i;
	struct list_head *p;
	CONFIG_RECORD_LIST *print;

	pr_debug("DDIC=====>:print_from_head_to_tail  START\n");

	list_for_each_prev(p, &head_list.list) {
		print = list_entry(p, CONFIG_RECORD_LIST, list);
		pr_debug("type:%d num %d value:\r\n", print->record.type, print->record.ins_num);
		for (i = 0; i < print->record.ins_num; i++)
			pr_debug("0x%x\t", print->record.ins_array[i]);
		pr_debug("\r\n");
	}
	pr_debug("DDIC=====>:print_from_head_to_tail  END\n");

}
#endif

static void free_list_memory(void)
{
	struct list_head *p, *n;
	CONFIG_RECORD_LIST *print;

	list_for_each_safe(p, n, &head_list.list) {
		print = list_entry(p, CONFIG_RECORD_LIST, list);
		list_del(&print->list);
		kfree(print);
	}
	/* test here : head->next == head ?? */
	if (list_empty(&head_list.list))
		pr_debug("*****list is empty!!\n");
	else
		pr_debug("*****list is NOT empty!!\n");

}

static int fbconfig_open(struct inode *inode, struct file *file)
{
	PM_TOOL_T *pm_params;

	file->private_data = inode->i_private;
	mutex_init(&fb_config_lock);
	pm_params = (PM_TOOL_T *) pm_get_handle();
	PanelMaster_set_PM_enable(1);
	pm_params->pLcm_drv = DISP_GetLcmDrv();
	pm_params->pLcm_params = DISP_GetLcmPara();

	return 0;
}


static char fbconfig_buffer[2048];

static ssize_t fbconfig_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	const int debug_bufmax = sizeof(fbconfig_buffer) - 1;	/* 2047 */
	int n = 0;

	n += scnprintf(fbconfig_buffer + n, debug_bufmax - n, "sxkhome");
	fbconfig_buffer[n++] = 0;
	/* n = 5 ; */
	/* memcpy(fbconfig_buffer,"sxkhome",6); */
	return simple_read_from_buffer(ubuf, count, ppos, fbconfig_buffer, n);
}

static ssize_t fbconfig_write(struct file *file,
			      const char __user *ubuf, size_t count, loff_t *ppos)
{
	return 0;
}


static long fbconfig_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	PM_TOOL_T *pm = (PM_TOOL_T *) pm_get_handle();
	uint32_t dsi_id = pm->dsi_id;
	LCM_DSI_PARAMS *pParams = get_dsi_params_handle(dsi_id);

	switch (cmd) {
	case GET_DSI_ID:
		{
			put_user(dsi_id, (unsigned long *)argp);
			return 0;
		}
	case SET_DSI_ID:
		{
			if (arg > PM_DSI_DUAL)
				return -EINVAL;
			pm->dsi_id = arg;
			pr_debug("fbconfig=>SET_DSI_ID:%d\n", dsi_id);

			return 0;
		}
	case LCM_TEST_DSI_CLK:
		{
			LCM_TYPE_FB lcm_fb;
			LCM_PARAMS *pLcm_params = pm->pLcm_params;

			lcm_fb.clock = pLcm_params->dsi.PLL_CLOCK;
			lcm_fb.lcm_type = pLcm_params->dsi.mode;

			pr_debug("fbconfig=>LCM_TEST_DSI_CLK:%d\n", ret);
			return copy_to_user(argp, &lcm_fb, sizeof(lcm_fb)) ? -EFAULT : 0;
		}
	case LCM_GET_ID:
		{
/* LCM_DRIVER*pLcm_drv=pm->pLcm_drv; */
			unsigned int lcm_id = 0;
#if 0
			if (pLcm_drv != NULL)
				lcm_id = pLcm_drv->get_lcm_id();
			else
				pr_debug("fbconfig=>LCM_GET_ID:%x\n", lcm_id);
#endif
			return copy_to_user(argp, &lcm_id, sizeof(lcm_id)) ? -EFAULT : 0;
		}
	case DRIVER_IC_CONFIG:
		{

			CONFIG_RECORD_LIST *record_tmp_list = kmalloc(sizeof(*record_tmp_list), GFP_KERNEL);

			if (copy_from_user
			    (&record_tmp_list->record, (void __user *)arg, sizeof(CONFIG_RECORD))) {
				pr_debug("list_add: copy_from_user failed! line:%d\n", __LINE__);
				return -EFAULT;
			}
			mutex_lock(&fb_config_lock);
			list_add(&record_tmp_list->list, &head_list.list);
			mutex_unlock(&fb_config_lock);
			return 0;
		}
	case DRIVER_IC_CONFIG_DONE:
		{
			/* print_from_head_to_tail(); */
			mutex_lock(&fb_config_lock);
			Panel_Master_dsi_config_entry("PM_DDIC_CONFIG", NULL);
			/*free the memory ..... */
			free_list_memory();
			mutex_unlock(&fb_config_lock);
			return 0;
		}
	case MIPI_SET_CC:
		{
			uint32_t enable = 0;

			if (get_user(enable, (uint32_t __user *) argp)) {
				pr_debug("[MIPI_SET_CC]: copy_from_user failed! line:%d\n",
				       __LINE__);
				return -EFAULT;
			}
			PanelMaster_set_CC(dsi_id, enable);
			return 0;
		}

	case LCM_GET_DSI_CONTINU:
		{
			uint32_t ret = PanelMaster_get_CC(dsi_id);
			/* need to improve ,0 now means nothing but one parameter.... */
			pr_debug("LCM_GET_DSI_CONTINU=>DSI: %d\n", ret);
			return put_user(ret, (unsigned long *)argp);
		}
	case MIPI_SET_CLK:
		{
			uint32_t clk = 0;

			if (get_user(clk, (uint32_t __user *) argp)) {
				pr_debug("[MIPI_SET_CLK]: copy_from_user failed! line:%d\n",
				       __LINE__);
				return -EFAULT;
			}
			pr_debug("LCM_GET_DSI_CLK=>dsi:%d\n", clk);
			Panel_Master_dsi_config_entry("PM_CLK", &clk);
			return 0;
		}
	case LCM_GET_DSI_CLK:
		{
			uint32_t clk = pParams->PLL_CLOCK;

			pr_debug("LCM_GET_DSI_CLK=>dsi:%d\n", clk);
			return put_user(clk, (unsigned long *)argp);
		}
	case MIPI_SET_SSC:
		{
			DSI_RET dsi_ssc;

			if (copy_from_user(&dsi_ssc, (void __user *)argp, sizeof(dsi_ssc))) {
				pr_debug("[MIPI_SET_SSC]: copy_from_user failed! line:%d\n",
				       __LINE__);
				return -EFAULT;
			}
			pr_debug("Pmaster:set mipi ssc line:%d\n", __LINE__);
			Panel_Master_dsi_config_entry("PM_SSC", &dsi_ssc);
			return 0;
		}

	case LCM_GET_DSI_SSC:
		{
			uint32_t ssc = pParams->ssc_range;

			if (pParams->ssc_disable)
				ssc = 0;
			return put_user(ssc, (unsigned long *)argp);
		}

	case LCM_GET_DSI_LANE_NUM:
		{
			uint32_t lane_num = pParams->LANE_NUM;

			pr_debug("Panel Master=>LCM_GET_DSI_Lane_num=>dsi:%d\r\n", lane_num);
			return put_user(lane_num, (unsigned long *)argp);
		}

	case LCM_GET_DSI_TE:
		{
			int ret;

			ret = PanelMaster_get_TE_status(dsi_id);
			pr_debug("fbconfig=>LCM_GET_DSI_TE:%d\n", ret);
			return put_user(ret, (unsigned long *)argp);
		}
	case LCM_GET_DSI_TIMING:
		{
			uint32_t ret;
			MIPI_TIMING timing;

			if (copy_from_user(&timing, (void __user *)argp, sizeof(timing))) {
				pr_debug("[MIPI_GET_TIMING]: copy_from_user failed! line:%d\n",
				       __LINE__);
				return -EFAULT;
			}
			ret = PanelMaster_get_dsi_timing(dsi_id, timing.type);
			pr_debug("fbconfig=>LCM_GET_DSI_TIMING:%d\n", ret);
			timing.value = ret;
			return copy_to_user(argp, &timing, sizeof(timing)) ? -EFAULT : 0;
		}

	case MIPI_SET_TIMING:
		{
			MIPI_TIMING timing;

			if (is_early_suspended)
				return -EFAULT;
			if (copy_from_user(&timing, (void __user *)argp, sizeof(timing))) {
				pr_debug("[MIPI_SET_TIMING]: copy_from_user failed! line:%d\n",
				       __LINE__);
				return -EFAULT;
			}
			PanelMaster_DSI_set_timing(dsi_id, timing);
			return 0;
		}
	case FB_LAYER_GET_EN:
		{
			PM_LAYER_EN layers;
			OVL_BASIC_STRUCT ovl_all[OVL_LAYER_NUM];

			ovl_get_info(0, ovl_all);
			layers.layer_en[0] = (ovl_all[0].layer_en ? 1 : 0);
			layers.layer_en[1] = (ovl_all[1].layer_en ? 1 : 0);
			layers.layer_en[2] = (ovl_all[2].layer_en ? 1 : 0);
			layers.layer_en[3] = (ovl_all[3].layer_en ? 1 : 0);
#ifdef OVL_CASCADE_SUPPORT
			layers.layer_en[4] = (ovl_all[4].layer_en ? 1 : 0);
			layers.layer_en[5] = (ovl_all[5].layer_en ? 1 : 0);
			layers.layer_en[6] = (ovl_all[6].layer_en ? 1 : 0);
			layers.layer_en[7] = (ovl_all[7].layer_en ? 1 : 0);
#endif
			pr_debug("[LAYER_GET_EN]:L0:%d L1:%d L2:%d L3:%d\n", ovl_all[0].layer_en,
			       ovl_all[1].layer_en, ovl_all[2].layer_en, ovl_all[3].layer_en);
			return copy_to_user(argp, &layers, sizeof(layers)) ? -EFAULT : 0;
		}
	case FB_LAYER_GET_INFO:
		{
			return 0;
		}
	case FB_LAYER_DUMP:
		{
			return 0;
		}
	case LCM_GET_ESD:
		{
			ESD_PARA esd_para;
			uint8_t *buffer;

			if (copy_from_user(&esd_para, (void __user *)arg, sizeof(esd_para))) {
				pr_debug("[LCM_GET_ESD]: copy_from_user failed! line:%d\n",
				       __LINE__);
				return -EFAULT;
			}
			buffer =  kzalloc(esd_para.para_num + 6, GFP_KERNEL);
			if (!buffer)
				return -ENOMEM;

			ret =
			    fbconfig_get_esd_check_test(dsi_id, esd_para.addr, buffer,
							esd_para.para_num + 6);
			if (ret < 0) {
				kfree(buffer);
				return -EFAULT;
			}
			ret =
				    copy_to_user(esd_para.esd_ret_buffer, buffer,
						 esd_para.para_num);
			kfree(buffer);
			return ret;


			return 0;
		}
	case TE_SET_ENABLE:
		{
			uint32_t te_enable = 0;

			if (get_user(te_enable, (unsigned long *)argp))
				return -EFAULT;

			return 0;
		}
	case DRIVER_IC_RESET:
		{
			Panel_Master_dsi_config_entry("DRIVER_IC_RESET", NULL);
			return 0;
		}
	default:
		return ret;
	}
}


static int fbconfig_release(struct inode *inode, struct file *file)
{
	PanelMaster_set_PM_enable(0);

	return 0;
}


static const struct file_operations fbconfig_fops = {
	.read = fbconfig_read,
	.write = fbconfig_write,
	.open = fbconfig_open,
	.unlocked_ioctl = fbconfig_ioctl,
	.release = fbconfig_release,
};

void PanelMaster_Init(void)
{
	ConfigPara_dbgfs = debugfs_create_file("fbconfig",
					       S_IFREG | S_IRUGO, NULL, (void *)0, &fbconfig_fops);

	INIT_LIST_HEAD(&head_list.list);
}

void PanelMaster_Deinit(void)
{
	debugfs_remove(ConfigPara_dbgfs);
}
