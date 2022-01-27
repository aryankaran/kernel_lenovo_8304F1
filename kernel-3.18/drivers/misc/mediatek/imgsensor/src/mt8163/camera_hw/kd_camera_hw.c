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
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_err(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...) pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)  printk(PFX  fmt, ##args)

#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif


/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam_mclk_h = NULL;
struct pinctrl_state *cam_mclk_l = NULL;
//struct pinctrl_state *cam_ldo0_h = NULL;
//struct pinctrl_state *cam_ldo0_l = NULL;

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	cam_mclk_h = pinctrl_lookup_state(camctrl, "cam_mclk1");
	if (IS_ERR(cam_mclk_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam_mclk_h\n", __func__);
	}


	cam_mclk_l = pinctrl_lookup_state(camctrl, "cam_mclk0");
	if (IS_ERR(cam_mclk_l)) {
		ret = PTR_ERR(cam_mclk_l);
		pr_debug("%s : pinctrl err, cam_mclk_l\n", __func__);
	}
	/*externel LDO enable */
	/*
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		pr_debug("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}


	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		pr_debug("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}*/
	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	switch (PwrType) {
	case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;
	case CAMPDN:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}

		break;
        case CAMMCLK:
                if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_mclk_l);
			else
				pinctrl_select_state(camctrl, cam_mclk_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_mclk_l);
			else
				pinctrl_select_state(camctrl, cam_mclk_h);
		}

		break;
	/*case CAMLDO:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
		break;*/
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}




int cntVCAMD = 0;
int cntVCAMA = 0;
int cntVCAMIO = 0;
int cntVCAMAF = 0;
int cntVCAMD_SUB = 0;

static DEFINE_SPINLOCK(kdsensor_pw_cnt_lock);


bool _hwPowerOnCnt(int PinIdx, KD_REGULATOR_TYPE_T powerId, int powerVolt, char *mode_name)
{
	if (_hwPowerOn(PinIdx, powerId, powerVolt)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD += 1;
		else if (powerId == VCAMA)
			cntVCAMA += 1;
		else if (powerId == VCAMIO)
			cntVCAMIO += 1;
		else if (powerId == VCAMIO)
			cntVCAMAF += 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB += 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

bool _hwPowerDownCnt(int PinIdx, KD_REGULATOR_TYPE_T powerId, char *mode_name)
{

	if (_hwPowerDown(PinIdx, powerId)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD -= 1;
		else if (powerId == VCAMA)
			cntVCAMA -= 1;
		else if (powerId == VCAMIO)
			cntVCAMIO -= 1;
		else if (powerId == VCAMAF)
			cntVCAMAF -= 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB -= 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

void checkPowerBeforClose(int PinIdx, char *mode_name)
{

	int i = 0;

	PK_DBG
	    ("[checkPowerBeforClose]cntVCAMD:%d, cntVCAMA:%d,cntVCAMIO:%d, cntVCAMAF:%d, cntVCAMD_SUB:%d,\n",
	     cntVCAMD, cntVCAMA, cntVCAMIO, cntVCAMAF, cntVCAMD_SUB);


	for (i = 0; i < cntVCAMD; i++)
		_hwPowerDown(PinIdx, VCAMD);
	for (i = 0; i < cntVCAMA; i++)
		_hwPowerDown(PinIdx, VCAMA);
	for (i = 0; i < cntVCAMIO; i++)
		_hwPowerDown(PinIdx, VCAMIO);
	for (i = 0; i < cntVCAMAF; i++)
		_hwPowerDown(PinIdx, VCAMAF);
	for (i = 0; i < cntVCAMD_SUB; i++)
		_hwPowerDown(PinIdx, SUB_VCAMD);

	cntVCAMD = 0;
	cntVCAMA = 0;
	cntVCAMIO = 0;
	cntVCAMAF = 0;
	cntVCAMD_SUB = 0;

}



int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On,
		       char *mode_name)
{

	u32 pinSetIdx = 0;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

	u32 pinSet[3][8] = {

		{CAMERA_CMRST_PIN,
		 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 CAMERA_CMPDN_PIN,
		 CAMERA_CMPDN_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{CAMERA_CMRST1_PIN,
		 CAMERA_CMRST1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 CAMERA_CMPDN1_PIN,
		 CAMERA_CMPDN1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 }
	};



	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	 else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	 else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;

if (On)
{
        if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC5025A_MIPI_RAW)))
        {
                PK_DBG("alexadd camera hw GC5025 power on +++++\n");
                //First Power Pin low and Reset Pin Low
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                        mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                mdelay(1);
                //VCAM_IO
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                mdelay(1);
                //VCAM_D
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMD, VOL_1200, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                mdelay(1);
                //VCAM_A
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_AF
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMAF, VOL_2800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                mdelay(1);
                mtkcam_gpio_set(0,CAMMCLK,1);
                ISP_MCLK1_EN(1);
                mdelay(1);
                //enable active sensor
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                                mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
                mdelay(1);
                PK_DBG("alexadd camera hw GC5025 power on -----\n");
        }
        else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC5025ABLX4P_MIPI_RAW)))
        {
                PK_DBG("alexadd camera hw GC5025BLX4P power on +++++\n");
                //First Power Pin low and Reset Pin Low
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                        mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                mdelay(1);
                // VCAM_IO
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                mdelay(1);
                //VCAM_D
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMD, VOL_1200, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                mdelay(1);
                //VCAM_A
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_AF
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMAF, VOL_2800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                mdelay(1);
                mtkcam_gpio_set(0,CAMMCLK,1);
                ISP_MCLK1_EN(1);
                mdelay(1);
                //enable active sensor
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                                mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
                mdelay(1);
                PK_DBG("alexadd camera hw GC5025BLX4P power on -----\n");
        }
        else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI556_MIPI_RAW)))
        {
            PK_DBG("alexadd camera hw HI556 power on +++++\n");
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                        mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
            mdelay(1);
            //VCAM_IO
            if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_A
            if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                        goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_D
            if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMD, VOL_1200, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_AF
            if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMAF, VOL_2800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
            }
            mtkcam_gpio_set(0,CAMMCLK,1);
            ISP_MCLK1_EN(1);
            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                    mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
            mdelay(1);
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                    mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
            mdelay(1);
            PK_DBG("alexadd camera hw HI556 power on -----\n");
        }
        else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI556QUN4P_MIPI_RAW)))
        {
            PK_DBG("alexadd camera hw HI556QUN4P power on +++++\n");
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                        mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
            mdelay(1);
            //VCAM_IO
            if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_A
            if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                        goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_D
            if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMD, VOL_1200, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
            }
            //VCAM_AF
            if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMAF, VOL_2800, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
            }
            mtkcam_gpio_set(0,CAMMCLK,1);
            ISP_MCLK1_EN(1);
            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                    mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
            mdelay(1);
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                    mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
            mdelay(1);
            PK_DBG("alexadd camera hw HI556QUN4P power on -----\n");
        }
        else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_SP2509_MIPI_RAW)))
        {
               PK_DBG("alexadd camera hw SP2509 power on +++++\n");
                //disable inactive sensor
                if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMPDN,pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
                if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                //IO
                if (_hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name) != TRUE) {
                        PK_ERR("[CAMERA SENSOR] Fail to enable D2 power\n");
                        goto _kdCISModulePowerOn_exit_;
                }
                //AVDD
                if (_hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name) != TRUE) {
                        PK_ERR("[CAMERA SENSOR] Fail to enable A power\n");
                        goto _kdCISModulePowerOn_exit_;
                }
                mtkcam_gpio_set(0, CAMMCLK,1);
                ISP_MCLK1_EN(1);
                mdelay(5);
                //enable active sensor
                if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMPDN,pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                mdelay(4);
                if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
                mdelay(5);
                PK_DBG("alexadd camera hw   SP2509 power on -----\n");
        }
        else if (pinSetIdx == 1 && currSensorName && ((strcmp(currSensorName, SENSOR_DRVNAME_GC2375_MIPI_RAW) == 0)))
        {
                PK_DBG("alexadd camera hw GC2375 power on +++++\n");
                //First Power Pin low and Reset Pin Low
                if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMPDN,pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
                if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                mdelay(1);
                //VCAM_IO
                if (_hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name) != TRUE) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d\n",VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                mdelay(1);
                //VCAM_A
                if (_hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name) != TRUE) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n",VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                mtkcam_gpio_set(0, CAMMCLK,1);
                ISP_MCLK1_EN(1);
                if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMPDN,pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
                PK_DBG("alexadd camera hw GC2375 power on -----\n");
        }
        else
        {
                /*
                if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name))
                        {
                        PK_DBG("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO), power id = %d\n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }*/
        }
}
else/* power OFF */
{
        if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC5025A_MIPI_RAW)))
	{
                PK_DBG("alexadd camera hw GC5025 power off +++++\n");
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                        mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                mtkcam_gpio_set(0,CAMMCLK,0);
                ISP_MCLK1_EN(0);
                // VCAM_A
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMA, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_D
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMD, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                // VCAM_IO
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_AF
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMAF, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                PK_DBG("alexadd camera hw GC5025 power off -----\n");
	}
    else if(pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC5025ABLX4P_MIPI_RAW)))
	{
                PK_DBG("alexadd camera hw GC5025BLX4P power off +++++\n");
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                        mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                mtkcam_gpio_set(0,CAMMCLK,0);
                ISP_MCLK1_EN(0);
                //VCAM_A
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMA, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_D
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMD, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_IO
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_AF
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMAF, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                PK_DBG("alexadd camera hw GC5025BLX4P power off -----\n");
	}
        else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI556_MIPI_RAW)))
        {
                PK_DBG("alexadd camera hw HI556 power off +++++\n");
                mdelay(1);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                        mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                mtkcam_gpio_set(0,CAMMCLK,0);
                ISP_MCLK1_EN(0);
                //VCAM_D
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMD, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_A
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMA, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_IO
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_AF
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMAF, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                PK_DBG("alexadd camera hw HI556 power off -----\n");
        }
        else if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_HI556QUN4P_MIPI_RAW)))
        {
                PK_DBG("alexadd camera hw HI556QUN4P power off +++++\n");
                mdelay(1);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
                        mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
                        mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                mtkcam_gpio_set(0,CAMMCLK,0);
                ISP_MCLK1_EN(0);
                //VCAM_D
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMD, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_A
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMA, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_IO
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_AF
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMAF, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
                        goto _kdCISModulePowerOn_exit_;
                }
                PK_DBG("alexadd camera hw HI556QUN4P power off -----\n");
        }
	else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_SP2509_MIPI_RAW)))
	{
                PK_DBG("alexadd camera hw SP2509 power off +++++\n");
                //Set Power Pin low and Reset Pin Low
                if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                mtkcam_gpio_set(0, CAMMCLK,0);
                ISP_MCLK1_EN(0);
                if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMPDN,pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
                //VCAM_A
                if (_hwPowerDownCnt(pinSetIdx, VCAMA, mode_name) != TRUE) {
                        PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A),power id= (%d)\n",VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_IO
                if (_hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name) != TRUE) {
                        PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO),power id = %d\n",VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                PK_DBG("alexadd camera hw SP2509 power off ------\n");
	}
	else if (pinSetIdx == 1 && currSensorName && ((strcmp(currSensorName, SENSOR_DRVNAME_GC2375_MIPI_RAW) == 0)))
	{
                PK_DBG("alexadd camera hw  GC2375 power off +++++\n");
                //Set Power Pin low and Reset Pin Low
                if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMPDN,pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
                //Set Reset Pin Low
                if (pinSet[pinSetIdx][IDX_PS_CMRST] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMRST,pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
                mtkcam_gpio_set(0, CAMMCLK,0);
                ISP_MCLK1_EN(0);
                //VCAM_A
                if (_hwPowerDownCnt(pinSetIdx, VCAMA, mode_name) != TRUE) {
                        PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id = %d\n",VCAMA);
                        goto _kdCISModulePowerOn_exit_;
                }
                //VCAM_IO
                if (_hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name) != TRUE) {
                        PK_DBG("[CAMERA SENSOR] Fail to OFF IO power (VCAM_IO), power id = %d\n",VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }
                if (pinSet[pinSetIdx][IDX_PS_CMPDN] != GPIO_CAMERA_INVALID)
                        mtkcam_gpio_set(pinSetIdx, CAMPDN,pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
                PK_DBG("alexadd camera hw  GC2375 power off -----\n");
	}
	else
        {
                /*
                if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
                        PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n", VCAMIO);
                        goto _kdCISModulePowerOn_exit_;
                }*/
	}
}

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;

}
EXPORT_SYMBOL(kdCISModulePowerOn);

