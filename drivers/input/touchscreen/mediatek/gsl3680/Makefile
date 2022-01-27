#include $(srctree)/drivers/misc/mediatek/imgsensor/src/Makefile.custom
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gsl3680/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/


obj-y    +=  gsl_point_id.o
$(obj)/gsl_point_id.o: $(srctree)/$(obj)/gsl_point_id 
	cp $(srctree)/$(obj)/gsl_point_id $(obj)/gsl_point_id.o
obj-y    +=  gsl_ts_driver.o

