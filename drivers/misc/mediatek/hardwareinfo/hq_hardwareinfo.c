/* Huaqin  Inc. (C) 2011. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("HUAQIN SOFTWARE")
 * RECEIVED FROM HUAQIN AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. HUAQIN EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES HUAQIN PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE HUAQIN SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN HUAQIN SOFTWARE. HUAQIN SHALL ALSO NOT BE RESPONSIBLE FOR ANY HUAQIN
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND HUAQIN'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE HUAQIN SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT HUAQIN'S OPTION, TO REVISE OR REPLACE THE HUAQIN SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * HUAQIN FOR SUCH HUAQIN SOFTWARE AT ISSUE.
 *

 */


/*******************************************************************************
* Dependency
*******************************************************************************/

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
#include <linux/mtd/mtd.h>
//#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/uaccess.h>


#include <mt-plat/mt_boot.h>
//#include <mt-plat/mtk_boot_common.h>

#include <linux/proc_fs.h>


#include "../lcm/inc/lcm_drv.h"
#include "../../../input/touchscreen/mediatek/tpd.h"
//#include "../../../mmc/host/mediatek/mt8167/mt_sd.h"
//#include "../../../mmc/host/mediatek/emmc_rpmb.h"

#define GET_RAMSIZE
#define GET_BAT_MANUFACTURER
//#define CHECK_EFUSE_IS_SBC

#ifdef GET_RAMSIZE
#include <linux/mm.h>
#endif

#define HARDWARE_INFO_VERSION   0x7502
//#define EMMC_INFO "/sys/devices/mtk-msdc.0/11230000.msdc0/mmc_host/mmc0/mmc0:0001/manfid"
#define EMMC_INFO "/sys/class/block/mmcblk0/device/manfid"

/*chenguanghe add for EMMC capacity,20170410*/
#define EMMC_SIZE "/sys/class/block/mmcblk0/size"

#define EMMC_NAME "/sys/class/block/mmcblk0/device/name"

#ifdef GET_BAT_MANUFACTURER
extern char *g_battery_manufacturer;
#endif
#define GET_TP_INFO
char *g_main_camera=NULL;
char *g_sub_camera=NULL;

const char *lcm_temp_name = NULL;

char *g_gsensor_name  = NULL;

//extern flashdev_info devinfo;
// char *g_flash_name=NULL;
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
extern char *saved_command_line;

/****************************************************************************** 
 * Debug configuration
******************************************************************************/
static unsigned int get_emmc_info(char *emmc_name)
{
	struct file *fp = NULL;
	unsigned int manfid;
	char buff[30] = {0};
	mm_segment_t fs;  
	loff_t pos;
	/*Start get emmc info*/
	fp = filp_open(EMMC_INFO, O_RDONLY, 0);

	if (IS_ERR(fp) || !fp->f_op)
	{
			printk("lpz emmc filp_open return NULL\n");
			return 0;
	}
	printk("lpz emmc read\n");
	fs = get_fs();	
	set_fs(KERNEL_DS);	
	pos = 0;  
	vfs_read(fp, buff, sizeof(buff), &pos);  
	set_fs(fs);	//recover execution environment
	filp_close(fp,NULL);
	printk("lpz read: %s\n", buff);
	sscanf(buff, "0x%06x\n", &manfid);

	/*Start get emmc name*/
	fp = filp_open(EMMC_NAME, O_RDONLY, 0);

	if (IS_ERR(fp) || !fp->f_op)
	{
			printk("lpz emmc filp_open return NULL\n");
			return 0;
	}
	printk("lpz emmc read\n");
	fs = get_fs();	
	set_fs(KERNEL_DS);	
	pos = 0;  
	vfs_read(fp, buff, sizeof(buff), &pos);  
	set_fs(fs);	//recover execution environment
	filp_close(fp,NULL);
	printk("lpz read: %s\n", buff);
	//sscanf(buff, "%s\n", emmc_name);
	strcpy(emmc_name,buff);
	if(!emmc_name)
	{
		printk("******g_emmc_name is NULL*********\n");
		goto exit;
	}
	printk("lpz read g_emmc_name: %s\n", emmc_name);
exit:
	return manfid;
}

/*Begin:chenguanghe add for EMMC capacity,20170410*/
static unsigned int get_emmc_capacity(void)
{
	struct file *fp = NULL;
	unsigned int emmc_cap;
	char buff[30] = {0};
	mm_segment_t fs;  
	loff_t pos;  
	fp = filp_open(EMMC_SIZE, O_RDONLY, 0);

	if (IS_ERR(fp) || !fp->f_op)
	{
			printk("Gino emmc filp_open return NULL\n");
			return 0;
	}
	printk("Gino emmc read capacity\n");
	fs = get_fs();	
	set_fs(KERNEL_DS);	
	pos = 0;  
	vfs_read(fp, buff, sizeof(buff), &pos);  
	set_fs(fs);	//recover execution environment
	filp_close(fp,NULL);
	
	sscanf(buff, "%d\n", &emmc_cap);

	printk("Gino read capacity : %d\n", emmc_cap);
	
	return emmc_cap;

}
/*End:chenguanghe add for EMMC capacity,20170410*/


//hardware info driver
#if 0
static ssize_t show_hardware_info(struct device *dev,struct device_attribute *attr, char *buf)
{   
    int ret_value = 1;
    int count = 0;

    //show lcm driver 
    printk("show_lcm_info return lcm name\n");
    count += sprintf(buf, "LCM Name:   ");//12 bytes
    if(lcm_drv->name)   
        count += sprintf(buf+count, "%s\n", lcm_drv->name); 
    else
        count += sprintf(buf+count,"no lcd name found\n");


    //show nand drvier name
    printk("show nand device name\n");
    count += sprintf(buf+count, "NAND Name:  ");//12 bytes
    if(devinfo.devciename) 
        count += sprintf(buf+count, "%s\n", devinfo.devciename);
    else
        count += sprintf(buf+count,"no nand name found\n");    


    //show Main Camera drvier name
    printk("show Main Camera device name\n");
    count += sprintf(buf+count, "Main Camera:");//12 bytes
    if(g_main_camera) 
        count += sprintf(buf+count, "%s\n", g_main_camera);
    else
        count += sprintf(buf+count,"pls turn on camera once\n");

    //show Sub Camera drvier name
    printk("show Sub camera name\n");
    count += sprintf(buf+count, "Sub Camera: ");//12 bytes
    if(g_sub_camera) 
        count += sprintf(buf+count, "%s\n", g_sub_camera);
    else
        count += sprintf(buf+count,"pls turn on camera once\n");

    
    //A20 using 6620
    //show WIFI
    printk("show WIFI name\n");
    count += sprintf(buf+count, "WIFI:       ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6620\n");
    else
        count += sprintf(buf+count,"no WIFI name found\n");

    printk("show Bluetooth name\n");
    count += sprintf(buf+count, "Bluetooth:  ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6620\n");
    else
        count += sprintf(buf+count,"no Bluetooth name found\n");

    printk("show GPS name\n");
    count += sprintf(buf+count, "GPS:        ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6620\n");
    else
        count += sprintf(buf+count,"no GPS name found\n");

    printk("show FM name\n");
    count += sprintf(buf+count, "FM:         ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6620\n");
    else
        count += sprintf(buf+count,"no FM name found\n");

    
    ret_value = count;


    return 0;
}

static ssize_t store_hardware_info(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

#endif

static ssize_t show_version(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    
    ret_value = sprintf(buf, "Version:    :0x%x\n", HARDWARE_INFO_VERSION);     

    return ret_value;
}
static ssize_t store_version(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}



//ret_value = sprintf(buf, "lcd name	:%s\n", lcm_temp_name); 
#if 0

static int strcmp1(unsigned char *s1, unsigned char *s2)
{
	while(*s1 && *s2)  
	{
		if(*s1 > *s2)
			return 1;
		else if(*s1 < *s2)
			return -1;
		else
		{
			++s1;
			++s2;
			continue;
		}
	}
	if(*s1)
		return 1;
	if(*s2)
		return -1;
	return 0;
}
#endif
static ssize_t show_lcm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
	int count = 0 ;
  if(lcm_temp_name != NULL)
  	{
  		count += sprintf(buf, "lcd name    :%s\n", lcm_temp_name);
	#if 0
		if (!strcmp1(lcm_temp_name,"ili9881c_dsi_vdo_txd"))
		{
			count += sprintf(buf+count, "  DXP  \n");
		}
		else
		{
			count += sprintf(buf+count, "  BOE  \n");
		}
	#endif
		ret_value = count ;
  	}
	else
    	ret_value = sprintf(buf, "lcd name    :%s\n", "not_found");  

    return ret_value;
}
static ssize_t store_lcm(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
#ifdef GET_TP_INFO

extern struct tpd_device  *tpd;
extern struct tpd_driver_t *g_tpd_drv;



static ssize_t show_ctp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    count += sprintf(buf + count, "ctp driver  :%s\n", g_tpd_drv->tpd_device_name);

    //count += sprintf(buf + count, "ctp version :0x%x\n", tpd->dev->id.version); 
    count += sprintf(buf + count, "ctp version :%x\n", tpd->hq_ctp_firmware_version); 
    count += sprintf(buf + count, "ctp Config_Version :%d,0x%x\n", tpd->hq_ctp_config_version,tpd->hq_ctp_config_version); 
    if(tpd->hq_ctp_module_id)
        count += sprintf(buf + count, "ctp modlue id  :%d\n", tpd->hq_ctp_module_id);
    if(tpd->hq_ctp_module_name)
        count += sprintf(buf + count, "ctp modlue  :%s\n", tpd->hq_ctp_module_name);
    
    ret_value = count;
    
    return ret_value;
}
static ssize_t store_ctp(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
#endif

static ssize_t show_main_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

   
    if(g_main_camera)
        ret_value = sprintf(buf , "main camera :%s\n", g_main_camera);
    else
        ret_value = sprintf(buf , "main camera :PLS TURN ON CAMERA FIRST\n");
    
    
    return ret_value;
}
static ssize_t store__main_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


static ssize_t show_sub_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

    if(g_sub_camera)
        ret_value = sprintf(buf , "sub camera  :%s\n", g_sub_camera);
    else
        ret_value = sprintf(buf , "sub camera  :PLS TURN ON CAMERA FIRST\n");
	
    return ret_value;
}
static ssize_t store_sub_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


//MT6620 names
static ssize_t show_wifi(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "wifi name   :MT6625L\n");
    return ret_value;
}
static ssize_t show_bt(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "bt name     :MT6625L\n");
    return ret_value;
}
static ssize_t show_gps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "GPS name    :MT6625L\n");
    return ret_value;
}
static ssize_t show_fm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "FM name     :MT6625L\n");
    return ret_value;
}




static ssize_t store_wifi(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
static ssize_t store_bt(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
static ssize_t store_gps(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
static ssize_t store_fm(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
#if 0
char *g_alsps_name  = NULL;
static ssize_t show_alsps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CONFIG_CUSTOM_KERNEL_ALSPS)
    if(g_alsps_name)
        ret_value = sprintf(buf, "AlSPS name  :%s\n",g_alsps_name);     
    else
        ret_value = sprintf(buf, "AlSPS name  :Not found\n"); 
    #else
    ret_value = sprintf(buf, "AlSPS name  :Not support ALSPS\n");   
    #endif
    return ret_value;
}

static ssize_t store_alsps(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
#endif
static ssize_t show_gsensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CONFIG_CUSTOM_KERNEL_ACCELEROMETER)
    if(g_gsensor_name)
        ret_value = sprintf(buf, "GSensor name:%s\n",g_gsensor_name);   
    else
        ret_value = sprintf(buf, "GSensor name:Not found\n"); 
    #else
    ret_value = sprintf(buf, "GSensor name:Not support GSensor\n");     
    #endif
    return ret_value;
}

static ssize_t store_gsensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

#if 0
char *g_msensor_name  = NULL;
static ssize_t show_msensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CONFIG_CUSTOM_KERNEL_MAGNETOMETER)
    if(g_msensor_name)
        ret_value = sprintf(buf, "MSensor name:%s\n",g_msensor_name);   
    else
        ret_value = sprintf(buf, "MSensor name:Not found\n"); 
    #else
    ret_value = sprintf(buf, "MSensor name:Not support MSensor\n");     
    #endif
    return ret_value;
}

static ssize_t store_msensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

char *g_Gyro_name  = NULL;
static ssize_t show_gyro(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if defined(CONFIG_CUSTOM_KERNEL_GYROSCOPE)
    if(g_Gyro_name)
        ret_value = sprintf(buf, "Gyro  name  :%s\n",g_Gyro_name);  
    else
        ret_value = sprintf(buf, "Gyro  name  :Not found\n"); 
    #else
    ret_value = sprintf(buf, "Gyro  name  :Not support Gyro\n");    
    #endif
    return ret_value;
}

static ssize_t store_gyro(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}
#endif
 
#ifdef GET_RAMSIZE
//extern u64 msdc_get_capacity(int get_emmc_total);
//extern unsigned int msdc_get_capacity(int get_emmc_total);

unsigned int round_kbytes_to_readable_mbytes(unsigned int k){
	unsigned int r_size_m = 0;

	if(k > 64*1024*1024){
		r_size_m = 128*1024;
	}else if(k > 32*1024*1024){
		r_size_m = 64*1024;
	}else if(k > 16*1024*1024){
		r_size_m = 32*1024;
	}else if(k > 8*1024*1024){
		r_size_m = 16*1024;
	}else if(k > 4*1024*1024){
		r_size_m = 8*1024;
	}else if(k > 3*1024*1024){
		r_size_m = 4*1024;
	}else if(k > 2*1024*1024){
		r_size_m = 3*1024;   //Modified by hubin, for support 3Gb LPDD, 20161101
	}else if(k > 1*1024*1024){
		r_size_m = 2*1024;
	}else if(k > 512*1024){
		r_size_m = 1024;
	}else if(k > 256*1024){
		r_size_m = 512;
	}else if(k > 128*1024){
		r_size_m = 256;
	}else{
		k = 0;
	}
	
	return r_size_m;
}

static ssize_t show_ramsize(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

	#define K(x) ((x) << (PAGE_SHIFT - 10))
    struct sysinfo i;
    si_meminfo(&i);

	if(round_kbytes_to_readable_mbytes(K(i.totalram)) >= 1024){
    	ret_value = sprintf(buf,"%dGB\n",round_kbytes_to_readable_mbytes(K(i.totalram))/1024);
    }else{
    	ret_value = sprintf(buf,"%dMB\n",round_kbytes_to_readable_mbytes(K(i.totalram)));
    }


    return ret_value;
}

static ssize_t show_flashsize(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    
    struct sysinfo i;
    si_meminfo(&i);

	/*Begin:chenguanghe add for EMMC capacity,20170410*/
    //ret_value = sprintf(buf,"%dGB",round_kbytes_to_readable_mbytes(msdc_get_capacity(1)/2)/1024);
    ret_value = sprintf(buf,"%dGB\n",round_kbytes_to_readable_mbytes(get_emmc_capacity()/2)/1024);
	/*End:chenguanghe add for EMMC capacity,20170410*/
	
    return ret_value;
}
#endif
//kaak_14_1113 end

static ssize_t show_flashname(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;
    unsigned int manfid = 0;
	
	char emmc_name[30] = {0};

    manfid = get_emmc_info(emmc_name);

//	printk("manfid=0x%x\n", manfid);
//	printk("g_emmc_name= %d\n",strncmp(emmc_name,"DF4016",6));
/*
#define CID_MANFID_SANDISK		0x2
#define CID_MANFID_TOSHIBA		0x11
#define CID_MANFID_MICRON		0x13
#define CID_MANFID_SAMSUNG		0x15
#define CID_MANFID_SANDISK_NEW	0x45
#define CID_MANFID_HYNIX			0x90
#define CID_MANFID_KSI			0x70
*/
    if (manfid == 0) 
	{
        count += sprintf(buf + count, "%s", "emmc :invild\n");
    } 
	else if (manfid == 0x90)
	{
		if(strncmp(emmc_name,"HAG4a2",6) == 0)
		{
        	count += sprintf(buf + count, "%s", "EMMC info:      HYNIX H9TQ17ABJTBCUR-KUM\n");//1961
		}

		else if(strncmp(emmc_name,"H8G4a2",6) == 0)
		{
			count += sprintf(buf + count, "%s", "EMMC info:      HYNIX H26M41208HPR\n");//1960
		}
		else
		{
			count += sprintf(buf + count, "%s", "EMMC info:      HYNIX \n");
		}
    } 
	else if (manfid == 0x15)
	{
		if(strncmp(emmc_name,"8GME4R",6) == 0)
		{
        	count += sprintf(buf + count, "%s", "EMMC info:      SAMSUNG KLM8G1GEME-B041\n");//1960
		}
		else if(strncmp(emmc_name,"AJTD4R",6) == 0)
		{
			count += sprintf(buf + count, "%s", "EMMC info:      SAMSUNG KLMAG1JETD-B041\n");//1960
		}
		else if(strncmp(emmc_name,"FE62MB",6) == 0)
		{
			count += sprintf(buf + count, "%s", "EMMC info:      SAMSUNG KMFE60012M-B214\n");//1961
		}
		else if(strncmp(emmc_name,"FE12MB",6) == 0)
		{
			count += sprintf(buf + count, "%s", "EMMC info:      SAMSUNG KMFE10012M-B214013\n");//1961
		}
		else if(strncmp(emmc_name,"QE63MB",6) == 0)
		{
			count += sprintf(buf + count, "%s", "EMMC info:      SAMSUNG KMQE60013M-B318\n");//1961
		}
		else
		{
			count += sprintf(buf + count, "%s", "EMMC info:      SAMSUNG \n");
		}
    } 
	else if (manfid == 0x11)
	{
        count += sprintf(buf + count, "%s", "EMMC info:      TOSHIBA \n");
    } 
	else if (manfid == 0x13)
	{
        count += sprintf(buf + count, "%s", "EMMC info:      MICRON \n");
	}
	else if ((manfid == 0x45)&&(strncmp(emmc_name,"DF4016",6) == 0))
	{
        count += sprintf(buf + count, "%s", "EMMC info:      SANDISK_NEW SDINADF4-16G-1001K\n");//1960
	}
	else 
	{
		count += sprintf(buf + count, "%s", "EMMC info not add in hardware yet!**\n");
	}
	
    ret_value = count;
	
    return ret_value;

}

/*Begin:add by hubin, for Secure Boot Check, 2016-11-11*/
#ifdef CHECK_EFUSE_IS_SBC
static ssize_t show_efuse_isSBC(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret_value = 1;
	ret_value = snprintf(buf , 2, strstr(saved_command_line ,"isSBC=")+6);
	
	return ret_value;
}
#endif
/*End:add by hubin, for Secure Boot Check, 2016-11-11*/

#ifdef GET_BAT_MANUFACTURER
static ssize_t show_battery_manufacturer(struct device *dev,struct device_attribute *attr, char *buf)
{
	int ret_value = 1;
	ret_value = sprintf(buf , "battery manufacturer: %s\n", g_battery_manufacturer);
	
	return ret_value;
}
#endif
static DEVICE_ATTR(99_version, 0644, show_version, store_version);
static DEVICE_ATTR(00_lcm, 0644, show_lcm, store_lcm);

#ifdef GET_TP_INFO
static DEVICE_ATTR(01_ctp, 0644, show_ctp, store_ctp);
#endif
static DEVICE_ATTR(02_main_camera, 0644, show_main_camera, store__main_camera);
static DEVICE_ATTR(03_sub_camera, 0644, show_sub_camera, store_sub_camera);

//static DEVICE_ATTR(04_flash, 0666, show_flash, store_flash);
static DEVICE_ATTR(05_gsensor, 0644, show_gsensor, store_gsensor);
#if 0
static DEVICE_ATTR(06_msensor, 0644, show_msensor, store_msensor);
static DEVICE_ATTR(08_gyro, 0644, show_gyro, store_gyro);


static DEVICE_ATTR(07_alsps, 0644, show_alsps, store_alsps);
#endif

static DEVICE_ATTR(09_wifi, 0644, show_wifi, store_wifi);
static DEVICE_ATTR(10_bt, 0644, show_bt, store_bt);
static DEVICE_ATTR(11_gps, 0644, show_gps, store_gps);
static DEVICE_ATTR(12_fm, 0644, show_fm, store_fm);

#ifdef GET_RAMSIZE
static DEVICE_ATTR(20_ramsize,0444,show_ramsize,NULL);
static DEVICE_ATTR(21_flashsize,0444,show_flashsize,NULL);
#endif
static DEVICE_ATTR(22_flashname,0444,show_flashname,NULL);


/*Add by hubin, for Secure Boot Check, 2016-11-11*/
#ifdef CHECK_EFUSE_IS_SBC
static DEVICE_ATTR(23_efuse_isSBC,0444,show_efuse_isSBC,NULL);
#endif
#ifdef GET_BAT_MANUFACTURER
static DEVICE_ATTR(24_battery_manufacturer,0444,show_battery_manufacturer,NULL);
#endif
///////////////////////////////////////////////////////////////////////////////////////////
//// hq factory apk support
///////////////////////////////////////////////////////////////////////////////////////////
#if 0

#define PROC_NAME1	"boot_status"
static struct proc_dir_entry *fts_proc_entry1;
extern enum boot_mode_t get_boot_mode_ex(void);//Huangyisong_add 20130823 start
static ssize_t hq_apk_debug_read1( struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	char *page = NULL;
	char *ptr = NULL;
	int len, err = -1;
	char buf[1];

	page = kmalloc(PAGE_SIZE, GFP_KERNEL);	
	if (!page) 
	{				
		kfree(page);			
		return -ENOMEM; 
	}
	printk("get_boot_mode_ex =%d\n", get_boot_mode_ex());
	ptr = page; 
	if (11 == get_boot_mode_ex())
	{
		buf[0] = 1;
	}
#if 0
	else if(12 == get_boot_mode_ex())
	{
		buf[0] = 2;
	}
	else if(13 == get_boot_mode_ex())// //demomode
	{
		buf[0] = 3;
	}
#endif
	else
	{
		buf[0] = 0;
	}

	//ptr += sprintf(ptr, "%c", buf[0]);
	
	ptr += sprintf(ptr, "%d\n", buf[0]);

	len = ptr - page;								
	if(*ppos >= len)
	{				
		kfree(page);			
		return 0;		
	}		
	err = copy_to_user(buffer,(char *)page,len);					
	*ppos += len;	
	if(err) 
	{				
		kfree(page);			
		return err; 	
	}		
	kfree(page);	
	return len; 	
}

static const struct file_operations fts_proc_fops = { 
    .write = NULL,
    .read = hq_apk_debug_read1,
};


int hq_apk_create_proc_boot_status(void )
{
	fts_proc_entry1 = proc_create(PROC_NAME1, 0777, NULL,&fts_proc_fops);
	if (NULL == fts_proc_entry1) {
		printk("hq_hardwareinfo proc_creat faild");
		return -ENOMEM;
	}
	return 0;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int HardwareInfo_driver_probe(struct platform_device *dev)   
{   
    int ret_device_file = 0;

    printk("** HardwareInfo_driver_probe!! **\n" );
    
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_00_lcm)) != 0) goto exit_error;
    #ifdef GET_TP_INFO
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_01_ctp)) != 0) goto exit_error;
    #endif
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_02_main_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_03_sub_camera)) != 0) goto exit_error;
  
    //if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_04_flash)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_05_gsensor)) != 0) goto exit_error;
  //  if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_06_msensor)) != 0) goto exit_error;
 //   if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_08_gyro)) != 0) goto exit_error;
//    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_07_alsps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_09_wifi)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_10_bt)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_11_gps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_12_fm)) != 0) goto exit_error;
    
   	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_99_version)) != 0) goto exit_error;     // create hq apk ftm proc file
#ifdef GET_RAMSIZE
	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_20_ramsize)) != 0) goto exit_error; 
	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_21_flashsize)) != 0) goto exit_error; 
#endif
	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_22_flashname)) != 0) goto exit_error; 
     /*Add by hubin, for Secure Boot Check, 2016-11-11*/
#ifdef CHECK_EFUSE_IS_SBC
	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_23_efuse_isSBC)) != 0) goto exit_error; 
#endif
#ifdef GET_BAT_MANUFACTURER
	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_24_battery_manufacturer)) != 0) goto exit_error; 
#endif
   //ret_device_file = hq_apk_create_proc_boot_status();

exit_error: 
    return ret_device_file;
}

static int HardwareInfo_driver_remove(struct platform_device *dev)
{
    printk("** HardwareInfo_drvier_remove!! **");

    device_remove_file(&(dev->dev), &dev_attr_00_lcm);
    #ifdef GET_TP_INFO
    device_remove_file(&(dev->dev), &dev_attr_01_ctp);
    #endif
    device_remove_file(&(dev->dev), &dev_attr_02_main_camera);
    device_remove_file(&(dev->dev), &dev_attr_03_sub_camera);
    
    //device_remove_file(&(dev->dev), &dev_attr_04_flash);
//    device_remove_file(&(dev->dev), &dev_attr_05_gsensor);
   // device_remove_file(&(dev->dev), &dev_attr_06_msensor);
    //device_remove_file(&(dev->dev), &dev_attr_08_gyro);
//  device_remove_file(&(dev->dev), &dev_attr_07_alsps);
  device_remove_file(&(dev->dev), &dev_attr_09_wifi);
  device_remove_file(&(dev->dev), &dev_attr_10_bt);
    device_remove_file(&(dev->dev), &dev_attr_11_gps);
    device_remove_file(&(dev->dev), &dev_attr_12_fm);
#ifdef GET_RAMSIZE
    device_remove_file(&(dev->dev), &dev_attr_20_ramsize);
    device_remove_file(&(dev->dev), &dev_attr_21_flashsize);
#endif
    device_remove_file(&(dev->dev), &dev_attr_22_flashname);
    /*Add by hubin, for Secure Boot Check, 2016-11-11*/
#ifdef CHECK_EFUSE_IS_SBC
	device_remove_file(&(dev->dev), &dev_attr_23_efuse_isSBC);
#endif
#ifdef GET_BAT_MANUFACTURER
    device_remove_file(&(dev->dev), &dev_attr_24_battery_manufacturer);
#endif
    device_remove_file(&(dev->dev), &dev_attr_99_version);
    return 0;
}





static struct platform_driver HardwareInfo_driver = {
    .probe      = HardwareInfo_driver_probe,
    .remove     = HardwareInfo_driver_remove,
    .driver     = {
        .name = "HardwareInfo",
    },
};

static struct platform_device HardwareInfo_device = {
    .name   = "HardwareInfo",
    .id     = -1,
};




static int __init HardwareInfo_mod_init(void)
{
    int ret = 0;


    ret = platform_device_register(&HardwareInfo_device);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_2;
    }
    

    ret = platform_driver_register(&HardwareInfo_driver);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_1;
    }

    goto ok_result;

    
fail_1:
    platform_driver_unregister(&HardwareInfo_driver);
fail_2:
    platform_device_unregister(&HardwareInfo_device);
ok_result:

    return ret;
}

/*****************************************************************************/
static void __exit HardwareInfo_mod_exit(void)
{
    platform_driver_unregister(&HardwareInfo_driver);
    platform_device_unregister(&HardwareInfo_device);
}
/*****************************************************************************/
module_init(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);
/*****************************************************************************/
MODULE_AUTHOR("Kaka Ni <nigang@huaqin.com>");
MODULE_DESCRIPTION("MT6575 Hareware Info driver");
MODULE_LICENSE("GPL");



