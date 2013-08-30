##########################################################################
modules name 	: bcm40181  
supported 	: WEXT 
information	:(so far anly added WEXT interface,will added nl80211 soon)
##########################################################################

DHDCFLAGS += -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2     \
	-DUNRELEASEDCHIP -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64           \
	-DDHD_FIRSTREAD=64 -DDHD_GPL -DDHD_SCHED -DBDC -DTOE \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DBCMSDIO -DBCMLXSDMMC \
	-DBCMPLATFORM_BUS -DOEM_ANDROID -DEMBEDDED_PLATFORM  \
	-DKEEP_ALIVE -DMMC_SDIO_ABORT \
	-DDHD_DEBUG
# -DSDTEST 
#-DDHD_DEBUG_TRAP \

DHDCFLAGS += -DCONFIG_HAS_EARLYSUSPEND
DHDCFLAGS += -DBCMWAPI_WPI
DHDCFLAGS += -DCSCAN
DHDCFLAGS += -DSOFTAP
DHDCFLAGS += -DCUSTOMER_HW_AMLOGIC
DHDCFLAGS += -DCONFIG_HAS_WAKELOCK

DHDCFLAGS += -Idrivers/amlogic/wifi/broadcm_40181 -Idrivers/amlogic/wifi/broadcm_40181/include


DHDOFILES = dhd_linux.o linux_osl.o bcmutils.o dhd_common.o dhd_custom_gpio.o \
	wl_iw.o siutils.o sbutils.o aiutils.o hndpmu.o bcmwifi.o dhd_sdio.o   \
	dhd_linux_sched.o dhd_cdc.o bcmsdh_sdmmc.o bcmsdh.o bcmsdh_linux.o    \
	bcmsdh_sdmmc_linux.o

DHDOFILES +=bcmevent.o wl_android.o wldev_common.o dhd_bta.o

ifeq ($(CONFIG_BCM40181_WIFI_CONTROL_FUNC),y)
DHDCFLAGS += -DCONFIG_WIFI_CONTROL_FUNC
endif
ifeq ($(CONFIG_BCM40181_DHD_USE_STATIC_BUF),y)
DHDCFLAGS += -DDHD_USE_STATIC_BUF
endif
ifeq ($(CONFIG_BCM40181_HW_OOB),y)
DHDCFLAGS += -DHW_OOB -DCUSTOM_OOB_GPIO_NUM=INT_GPIO_4
endif
ifeq ($(CONFIG_BCM40181_OOB_INTR_ONLY),y)
DHDCFLAGS += -DOOB_INTR_ONLY -DCUSTOM_OOB_GPIO_NUM=INT_GPIO_4
else
DHDCFLAGS += -DSDIO_ISR_THREAD 
endif
ifeq ($(CONFIG_BCM40181_GET_CUSTOM_MAC_ENABLE),y)
DHDCFLAGS += -DGET_CUSTOM_MAC_ENABLE
endif

obj-$(CONFIG_SDIO_DHD_CDC_WIFI_40181_MODULE) += dhd.o
dhd-objs += $(DHDOFILES)
EXTRA_CFLAGS = $(DHDCFLAGS)
EXTRA_LDFLAGS += --strip-debug
