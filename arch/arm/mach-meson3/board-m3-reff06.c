/*
 *
 * arch/arm/mach-meson/meson.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/spi/flash.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/memory.h>
#include <mach/clock.h>
#include <mach/usbclock.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/lm.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <mach/nand.h>
#include <linux/i2c.h>
#include <linux/i2c-aml.h>
#include <mach/power_gate.h>
#include <linux/aml_bl.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <mach/card_io.h>
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <mach/clk_set.h>
#include "board-m3-reff06.h"

#ifdef CONFIG_SENSORS_MMC328X
#include <linux/mmc328x.h>
#endif
#ifdef CONFIG_SENSORS_MMA8452
#include <linux/mma8452.h>
#endif
#ifdef CONFIG_AW_AXP
#include <linux/power_supply.h>
#include <linux/apm_bios.h>
#include <linux/apm-emulation.h>
#include <linux/regulator/machine.h>
#include <mach/irqs.h>
#include "../../../drivers/amlogic/power/axp_power/axp-gpio.h"
#include "../../../drivers/amlogic/power/axp_power/axp-mfd.h"
#include "../../../drivers/amlogic/power/axp_power/axp-cfg.h"
#endif

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#ifdef CONFIG_SUSPEND
#include <mach/pm.h>
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
#include <media/amlogic/aml_camera.h>
#include <linux/camera/amlogic_camera_common.h>
#endif

#ifdef CONFIG_EFUSE
#include <linux/efuse.h>
#endif

#ifdef CONFIG_AW_AXP
extern void axp_power_off(void);
#endif

#ifdef CONFIG_AW_AXP20

int pmu_used;
int pmu_twi_id;
int pmu_irq_id;
int pmu_twi_addr;
int pmu_battery_rdc;
int pmu_battery_cap;
int pmu_init_chgcur;
int pmu_suspend_chgcur;
int pmu_resume_chgcur;
int pmu_shutdown_chgcur;
int pmu_init_chgvol;
int pmu_init_chgend_rate;
int pmu_init_chg_enabled;
int pmu_init_adc_freq;
int pmu_init_adc_freqc;
int pmu_init_chg_pretime;
int pmu_init_chg_csttime;

int pmu_bat_para1;
int pmu_bat_para2;
int pmu_bat_para3;
int pmu_bat_para4;
int pmu_bat_para5;
int pmu_bat_para6;
int pmu_bat_para7;
int pmu_bat_para8;
int pmu_bat_para9;
int pmu_bat_para10;
int pmu_bat_para11;
int pmu_bat_para12;
int pmu_bat_para13;
int pmu_bat_para14;
int pmu_bat_para15;
int pmu_bat_para16;

int pmu_usbvol_limit;
int pmu_usbvol;
int pmu_usbcur_limit;
int pmu_usbcur;

int pmu_pwroff_vol;
int pmu_pwron_vol;

int dcdc2_vol;
int dcdc3_vol;
int ldo2_vol;
int ldo3_vol;
int ldo4_vol;

int pmu_pekoff_time;
int pmu_pekoff_en;
int pmu_peklong_time;
int pmu_pekon_time;
int pmu_pwrok_time;
int pmu_pwrnoe_time;
int pmu_intotp_en;

/* Reverse engineered partly from Platformx drivers */
enum axp_regls{

	vcc_ldo1,
	vcc_ldo2,
	vcc_ldo3,
	vcc_ldo4,
	vcc_ldo5,

	vcc_buck2,
	vcc_buck3,
	vcc_ldoio0,
};

/* The values of the various regulator constraints are obviously dependent
 * on exactly what is wired to each ldo.  Unfortunately this information is
 * not generally available.  More information has been requested from Xbow
 * but as of yet they haven't been forthcoming.
 *
 * Some of these are clearly Stargate 2 related (no way of plugging
 * in an lcd on the IM2 for example!).
 */

static struct regulator_consumer_supply ldo1_data[] = {
		{
			.supply = "axp20_rtc",
		},
	};


static struct regulator_consumer_supply ldo2_data[] = {
		{
			.supply = "axp20_analog/fm",
		},
	};

static struct regulator_consumer_supply ldo3_data[] = {
		{
			.supply = "axp20_pll",
		},
	};

static struct regulator_consumer_supply ldo4_data[] = {
		{
			.supply = "axp20_hdmi",
		},
	};

static struct regulator_consumer_supply ldoio0_data[] = {
		{
			.supply = "axp20_mic",
		},
	};


static struct regulator_consumer_supply buck2_data[] = {
		{
			.supply = "axp20_core",
		},
	};

static struct regulator_consumer_supply buck3_data[] = {
		{
			.supply = "axp20_ddr",
		},
	};



static struct regulator_init_data axp_regl_init_data[] = {
	[vcc_ldo1] = {
		.constraints = { /* board default 1.25V */
			.name = "axp20_ldo1",
			.min_uV =  AXP20LDO1 * 1000,
			.max_uV =  AXP20LDO1 * 1000,
		},
		.num_consumer_supplies = ARRAY_SIZE(ldo1_data),
		.consumer_supplies = ldo1_data,
	},
	[vcc_ldo2] = {
		.constraints = { /* board default 3.0V */
			.name = "axp20_ldo2",
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.initial_state = PM_SUSPEND_STANDBY,
			.state_standby = {
				//.uV = ldo2_vol * 1000,
				.enabled = 1,
			}
		},
		.num_consumer_supplies = ARRAY_SIZE(ldo2_data),
		.consumer_supplies = ldo2_data,
	},
	[vcc_ldo3] = {
		.constraints = {/* default is 1.8V */
			.name = "axp20_ldo3",
			.min_uV =  700 * 1000,
			.max_uV =  3500* 1000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.initial_state = PM_SUSPEND_STANDBY,
			.state_standby = {
				//.uV = ldo3_vol * 1000,
				.enabled = 1,
			}
		},
		.num_consumer_supplies = ARRAY_SIZE(ldo3_data),
		.consumer_supplies = ldo3_data,
	},
	[vcc_ldo4] = {
		.constraints = {
			/* board default is 3.3V */
			.name = "axp20_ldo4",
			.min_uV = 1250000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.initial_state = PM_SUSPEND_STANDBY,
			.state_standby = {
				//.uV = ldo4_vol * 1000,
				.enabled = 1,
			}
		},
		.num_consumer_supplies = ARRAY_SIZE(ldo4_data),
		.consumer_supplies = ldo4_data,
	},
	[vcc_buck2] = {
		.constraints = { /* default 1.24V */
			.name = "axp20_buck2",
			.min_uV = 700 * 1000,
			.max_uV = 2275 * 1000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.initial_state = PM_SUSPEND_STANDBY,
			.state_standby = {
				//.uV = dcdc2_vol * 1000,
				.enabled = 1,
			}
		},
		.num_consumer_supplies = ARRAY_SIZE(buck2_data),
		.consumer_supplies = buck2_data,
	},
	[vcc_buck3] = {
		.constraints = { /* default 2.5V */
			.name = "axp20_buck3",
			.min_uV = 700 * 1000,
			.max_uV = 3500 * 1000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.initial_state = PM_SUSPEND_STANDBY,
			.state_standby = {
				//.uV = dcdc3_vol * 1000,
				.enabled = 1,
			}
		},
		.num_consumer_supplies = ARRAY_SIZE(buck3_data),
		.consumer_supplies = buck3_data,
	},
	[vcc_ldoio0] = {
		.constraints = { /* default 2.5V */
			.name = "axp20_ldoio0",
			.min_uV = 1800 * 1000,
			.max_uV = 3300 * 1000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ldoio0_data),
		.consumer_supplies = ldoio0_data,
	},
};

static struct axp_funcdev_info axp_regldevs[] = {
	{
		.name = "axp20-regulator",
		.id = AXP20_ID_LDO1,
		.platform_data = &axp_regl_init_data[vcc_ldo1],
	}, {
		.name = "axp20-regulator",
		.id = AXP20_ID_LDO2,
		.platform_data = &axp_regl_init_data[vcc_ldo2],
	}, {
		.name = "axp20-regulator",
		.id = AXP20_ID_LDO3,
		.platform_data = &axp_regl_init_data[vcc_ldo3],
	}, {
		.name = "axp20-regulator",
		.id = AXP20_ID_LDO4,
		.platform_data = &axp_regl_init_data[vcc_ldo4],
	}, {
		.name = "axp20-regulator",
		.id = AXP20_ID_BUCK2,
		.platform_data = &axp_regl_init_data[vcc_buck2],
	}, {
		.name = "axp20-regulator",
		.id = AXP20_ID_BUCK3,
		.platform_data = &axp_regl_init_data[vcc_buck3],
	}, {
		.name = "axp20-regulator",
		.id = AXP20_ID_LDOIO0,
		.platform_data = &axp_regl_init_data[vcc_ldoio0],
	},
};

static struct power_supply_info battery_data ={
		.name ="PTI PL336078",
		.technology = POWER_SUPPLY_TECHNOLOGY_LiFe,
		//.voltage_max_design = pmu_init_chgvol,
		//.voltage_min_design = pmu_pwroff_vol,
		//.energy_full_design = pmu_battery_cap,
		.use_for_apm = 1,
};


static struct axp_supply_init_data axp_sply_init_data = {
	.battery_info = &battery_data,
	//.chgcur = pmu_init_chgcur,
	//.chgvol = pmu_init_chgvol,
	//.chgend = pmu_init_chgend_rate,
	//.chgen = pmu_init_chg_enabled,
	//.sample_time = pmu_init_adc_freq,
	//.chgpretime = pmu_init_chg_pretime,
	//.chgcsttime = pmu_init_chg_csttime,
};

static struct axp_funcdev_info axp_splydev[]={
   	{
   		.name = "axp20-supplyer",
			.id = AXP20_ID_SUPPLY,
      .platform_data = &axp_sply_init_data,
    },
};

static struct axp_funcdev_info axp_gpiodev[]={
   	{   .name = "axp20-gpio",
   		.id = AXP20_ID_GPIO,
    },
};

static struct axp_platform_data axp_pdata = {
	.num_regl_devs = ARRAY_SIZE(axp_regldevs),
	.num_sply_devs = ARRAY_SIZE(axp_splydev),
	.num_gpio_devs = ARRAY_SIZE(axp_gpiodev),
	.regl_devs = axp_regldevs,
	.sply_devs = axp_splydev,
	.gpio_devs = axp_gpiodev,
	.gpio_base = 0,
};

#endif

#if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_KEYPADS_AM_MODULE)
static struct resource intput_resources[] = {
    {
        .start = 0x0,
        .end = 0x0,
        .name="8726",
        .flags = IORESOURCE_IO,
    },
};

static struct platform_device input_device = {
    .name = "m1-kp",
    .id = 0,
    .num_resources = ARRAY_SIZE(intput_resources),
    .resource = intput_resources,
    
};
#endif

#ifdef CONFIG_SARADC_AM
#include <linux/saradc.h>
static struct platform_device saradc_device = {
    .name = "saradc",
    .id = 0,
    .dev = {
        .platform_data = NULL,
    },
};
#endif

#ifdef CONFIG_ANDROID_TIMED_GPIO	//add by sz.wu.zhu
#ifndef _LINUX_TIMED_GPIO_H
#define _LINUX_TIMED_GPIO_H

#define TIMED_GPIO_NAME "timed-gpio"
struct timed_gpio {
	const char *name;
	unsigned 	gpio;
	int		max_timeout;
	u8 		active_low;
};

struct timed_gpio_platform_data {
	int 		num_gpios;
	struct timed_gpio *gpios;
};
#endif

static struct timed_gpio amlogic_gpio_vibravor_gpios[] ={
	{
		.name	="vibrator",
		.gpio	=( GPIOD_bank_bit0_9(2) << 16 )|GPIOD_bit_bit0_9(2),	//gpiod_2
		.max_timeout	= 15000,											//15s
		.active_low	= 1,
	},
};

static struct timed_gpio_platform_data amlogic_gpio_vibravor_data= {
	.num_gpios	= 1,
	.gpios		= amlogic_gpio_vibravor_gpios,
};

static struct platform_device amlogic_gpio_vibravor = {
	.name = TIMED_GPIO_NAME,
	.dev={
		.platform_data= &amlogic_gpio_vibravor_data,
	},
};
#endif

#ifdef CONFIG_ADC_TOUCHSCREEN_AM
#include <linux/adc_ts.h>

static struct adc_ts_platform_data adc_ts_pdata = {
    .irq = -1,  //INT_SAR_ADC
    .x_plate_ohms = 400,
};

static struct platform_device adc_ts_device = {
    .name = "adc_ts",
    .id = 0,
    .dev = {
        .platform_data = &adc_ts_pdata,
    },
};
#endif

#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
#include <linux/input.h>
#include <linux/adc_keypad.h>

static struct adc_key adc_kp_key[] = {
    {KEY_MENU,          "menu", CHAN_4, 0, 60},
    {KEY_VOLUMEDOWN,    "vol-", CHAN_4, 140, 60},
    {KEY_VOLUMEUP,      "vol+", CHAN_4, 266, 60},
    {KEY_HOME,          "home", CHAN_4, 386, 60},
    {KEY_BACK,          "exit", CHAN_4, 508, 60},
    {KEY_ENTER,          "enter", CHAN_4, 620, 60},
    {KEY_SEARCH,        "search", CHAN_4, 763, 60},
};

static struct adc_kp_platform_data adc_kp_pdata = {
    .key = &adc_kp_key[0],
    .key_num = ARRAY_SIZE(adc_kp_key),
};

static struct platform_device adc_kp_device = {
    .name = "m1-adckp",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
    .platform_data = &adc_kp_pdata,
    }
};
#endif

#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
#include <linux/input.h>
#include <linux/input/key_input.h>

int _key_code_list[] = {KEY_POWER};

static inline int key_input_init_func(void)
{
    WRITE_AOBUS_REG(AO_RTC_ADDR0, (READ_AOBUS_REG(AO_RTC_ADDR0) &~(1<<11)));
    WRITE_AOBUS_REG(AO_RTC_ADDR1, (READ_AOBUS_REG(AO_RTC_ADDR1) &~(1<<3)));
    return 0;
}
static inline int key_scan(int *key_state_list)
{
    int ret = 0;
    key_state_list[0] = ((READ_AOBUS_REG(AO_RTC_ADDR1) >> 2) & 1) ? 0 : 1;
    return ret;
}

static  struct key_input_platform_data  key_input_pdata = {
    .scan_period = 20,
    .fuzz_time = 60,
    .key_code_list = &_key_code_list[0],
    .key_num = ARRAY_SIZE(_key_code_list),
    .scan_func = key_scan,
    .init_func = key_input_init_func,
    .config = 0,
};

static struct platform_device input_device_key = {
    .name = "m1-keyinput",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &key_input_pdata,
    }
};
#endif


#if defined(CONFIG_FB_AM)
static struct resource fb_device_resources[] = {
    [0] = {
        .start = OSD1_ADDR_START,
        .end   = OSD1_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
#if defined(CONFIG_FB_OSD2_ENABLE)
    [1] = {
        .start = OSD2_ADDR_START,
        .end   = OSD2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
#endif
};

static struct platform_device fb_device = {
    .name       = "mesonfb",
    .id         = 0,
    .num_resources = ARRAY_SIZE(fb_device_resources),
    .resource      = fb_device_resources,
};
#endif
#ifdef CONFIG_USB_PHY_CONTROL
static struct resource usb_phy_control_device_resources[] = {
	{
		.start = CBUS_REG_ADDR(PREI_USB_PHY_REG),
		.end = -1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device usb_phy_control_device = {
	.name = "usb_phy_control",
	.id = -1,
	.resource = usb_phy_control_device_resources,
};
#endif
#ifdef CONFIG_USB_DWC_OTG_HCD
//usb_a is slave port
static struct lm_device usb_ld_a = {
    .type = LM_DEVICE_TYPE_USB,
    .id = 0,
    .irq = INT_USB_A,
    .resource.start = IO_USB_A_BASE,
    .resource.end = -1,
    .dma_mask_room = DMA_BIT_MASK(32),
    .port_type = USB_PORT_TYPE_SLAVE,
    .port_speed = USB_PORT_SPEED_DEFAULT,
    .dma_config = USB_DMA_BURST_SINGLE,
    .set_vbus_power = 0,
};

static struct lm_device usb_ld_b = {
    .type = LM_DEVICE_TYPE_USB,
    .id = 1,
    .irq = INT_USB_B,
    .resource.start = IO_USB_B_BASE,
    .resource.end = -1,
    .dma_mask_room = DMA_BIT_MASK(32),
    .port_type = USB_PORT_TYPE_HOST,
    .port_speed = USB_PORT_SPEED_DEFAULT,
    .dma_config = USB_DMA_BURST_SINGLE , //   USB_DMA_DISABLE,
    .set_vbus_power = 0,
};

#endif
#ifdef CONFIG_SATA_DWC_AHCI
static struct lm_device sata_ld = {
    .type = LM_DEVICE_TYPE_SATA,
    .id = 2,
    .irq = INT_SATA,
    .dma_mask_room = DMA_BIT_MASK(32),
    .resource.start = IO_SATA_BASE,
    .resource.end = -1,
};
#endif

#if defined(CONFIG_AM_STREAMING)
static struct resource codec_resources[] = {
    [0] = {
        .start =  CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = STREAMBUF_ADDR_START,
	 .end = STREAMBUF_ADDR_END,
	 .flags = IORESOURCE_MEM,
    },
};

static struct platform_device codec_device = {
    .name       = "amstream",
    .id         = 0,
    .num_resources = ARRAY_SIZE(codec_resources),
    .resource      = codec_resources,
};
#endif

#if defined(CONFIG_AM_DEINTERLACE) || defined (CONFIG_DEINTERLACE)
static struct resource deinterlace_resources[] = {
    [0] = {
        .start =  DI_ADDR_START,
        .end   = DI_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device deinterlace_device = {
    .name       = "deinterlace",
    .id         = 0,
    .num_resources = ARRAY_SIZE(deinterlace_resources),
    .resource      = deinterlace_resources,
};
#endif

#if defined(CONFIG_TVIN_VDIN)
static struct resource vdin_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,  //pbufAddr
        .end   = VDIN_ADDR_END,     //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = VDIN_ADDR_START,
        .end   = VDIN_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [2] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
    [3] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device vdin_device = {
    .name       = "vdin",
    .id         = -1,
    .num_resources = ARRAY_SIZE(vdin_resources),
    .resource      = vdin_resources,
};
#endif

#ifdef CONFIG_TVIN_BT656IN
static struct platform_device bt656in_device = {
    .name       = "amvdec_656in",
    .id         = -1,
};
#endif

#if defined(CONFIG_CARDREADER)
static struct resource amlogic_card_resource[] = {
    [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x120024c,
        .flags = 0x200,
    }
};

/* WIFI ON Flag */
static int WIFI_ON;
/* BT ON Flag */
static int BT_ON;
/* WL_BT_REG_ON control function */
static void reg_on_control(int is_on)
{ 	 
  CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0,(1<<29)); 
  CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_3,(1<<26));
  CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7,(1<<16));
  
  set_gpio_mode(GPIOD_bank_bit0_9(8), GPIOD_bit_bit0_9(8), GPIO_OUTPUT_MODE);   
  if(is_on){
    set_gpio_val(GPIOD_bank_bit0_9(8), GPIOD_bit_bit0_9(8), 1);
  }
  else{
    /* only pull donw reg_on pin when wifi and bt off */
    if((!WIFI_ON) && (!BT_ON)){
      set_gpio_val(GPIOD_bank_bit0_9(8), GPIOD_bit_bit0_9(8), 0);
      printk(KERN_INFO "WIFI BT Power down\n");
    }
	}
}

void extern_wifi_power(int is_power)
{
  WIFI_ON = is_power;
  reg_on_control(is_power);
}

EXPORT_SYMBOL(extern_wifi_power);

#define GPIO_WIFI_HOSTWAKE  ((GPIOX_bank_bit0_31(11)<<16) |GPIOX_bit_bit0_31(11))

void sdio_extern_init(void)
{
	/* set reset pin(GPIOD_7) high */
	set_gpio_mode(GPIOD_bank_bit0_9(7), GPIOD_bit_bit0_9(7), GPIO_OUTPUT_MODE);   
	set_gpio_val(GPIOD_bank_bit0_9(7), GPIOD_bit_bit0_9(7), 1);
	
  #if defined(CONFIG_BCM4329_HW_OOB) || defined(CONFIG_BCM4329_OOB_INTR_ONLY)/* Jone add */
    gpio_direction_input(GPIO_WIFI_HOSTWAKE);
    gpio_enable_level_int(gpio_to_idx(GPIO_WIFI_HOSTWAKE), 0, 4);
    gpio_enable_edge_int(gpio_to_idx(GPIO_WIFI_HOSTWAKE), 0, 4);
  #endif /* (CONFIG_BCM4329_HW_OOB) || (CONFIG_BCM4329_OOB_INTR_ONLY) Jone add */
  
  extern_wifi_power(1);
}

static struct aml_card_info  amlogic_card_info[] = {
    [0] = {
        .name = "sd_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_B_CARD_0_5,
        .card_ins_en_reg = CARD_GPIO_ENABLE,
        .card_ins_en_mask = PREG_IO_29_MASK,
        .card_ins_input_reg = CARD_GPIO_INPUT,
        .card_ins_input_mask = PREG_IO_29_MASK,
        .card_power_en_reg = CARD_GPIO_ENABLE,
        .card_power_en_mask = PREG_IO_31_MASK,
        .card_power_output_reg = CARD_GPIO_OUTPUT,
        .card_power_output_mask = PREG_IO_31_MASK,
        .card_power_en_lev = 0,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = 0,
    },
    [1] = {
        .name = "sdio_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_A_GPIOX_0_3,
        .card_ins_en_reg = 0,
        .card_ins_en_mask = 0,
        .card_ins_input_reg = 0,
        .card_ins_input_mask = 0,
        .card_power_en_reg = 0,
        .card_power_en_mask = 0,
        .card_power_output_reg = 0,
        .card_power_output_mask = 0,
        .card_power_en_lev = 1,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = sdio_extern_init,
    },
};

void extern_wifi_reset(int is_on)
{
  unsigned int val;

  set_gpio_mode(GPIOD_bank_bit0_9(7), GPIOD_bit_bit0_9(7), GPIO_OUTPUT_MODE);   
  if(is_on){
		set_gpio_val(GPIOD_bank_bit0_9(7), GPIOD_bit_bit0_9(7), 1);
  }
	else{
		set_gpio_val(GPIOD_bank_bit0_9(7), GPIOD_bit_bit0_9(7), 0);
	} 		
}
EXPORT_SYMBOL(extern_wifi_reset);

static struct aml_card_platform amlogic_card_platform = {
    .card_num = ARRAY_SIZE(amlogic_card_info),
    .card_info = amlogic_card_info,
};

static struct platform_device amlogic_card_device = { 
    .name = "AMLOGIC_CARD", 
    .id    = -1,
    .num_resources = ARRAY_SIZE(amlogic_card_resource),
    .resource = amlogic_card_resource,
    .dev = {
        .platform_data = &amlogic_card_platform,
    },
};

#endif

#if defined(CONFIG_AML_AUDIO_DSP)
static struct resource audiodsp_resources[] = {
    [0] = {
        .start = AUDIODSP_ADDR_START,
        .end   = AUDIODSP_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device audiodsp_device = {
    .name       = "audiodsp",
    .id         = 0,
    .num_resources = ARRAY_SIZE(audiodsp_resources),
    .resource      = audiodsp_resources,
};
#endif

static struct resource aml_m3_audio_resource[] = {
    [0] =   {
        .start  =   0,
        .end        =   0,
        .flags  =   IORESOURCE_MEM,
    },
};

/* Check current mode, 0: panel; 1: !panel*/
int get_display_mode(void) {
	int fd;
	int ret = 0;
	char mode[8];	
	
	fd = sys_open("/sys/class/display/mode", O_RDWR | O_NDELAY, 0);
	if(fd >= 0) {
	  	memset(mode,0,8);
	  	sys_read(fd,mode,8);
	  	if(strncmp("panel",mode,5))
	  		ret = 1;
	  	sys_close(fd);
	}

	return ret;
}

#if defined(CONFIG_SND_AML_M3)
static struct platform_device aml_audio = {
    .name               = "aml_m3_audio",
    .id                     = -1,
    .resource       =   aml_m3_audio_resource,
    .num_resources  =   ARRAY_SIZE(aml_m3_audio_resource),
};

int aml_m3_is_hp_pluged(void)
{
	if(get_display_mode() != 0) //if !panel, return 1 to mute spk		
    return 1;
		
	return READ_CBUS_REG_BITS(PREG_PAD_GPIO0_I, 19, 1); //return 1: hp pluged, 0: hp unpluged.
}

struct aml_m3_platform_data {
  int (*is_hp_pluged)(void);
};

void mute_spk(struct snd_soc_codec* codec, int flag)
{
  printk(KERN_DEBUG "***Entered %s:%s\n", __FILE__,__func__);
    
  if(flag){
    set_gpio_val(GPIOD_bank_bit0_9(4), GPIOD_bit_bit0_9(4), 0);	 // mute speak
    set_gpio_mode(GPIOD_bank_bit0_9(4), GPIOD_bit_bit0_9(4), GPIO_OUTPUT_MODE);
  }else{
    set_gpio_val(GPIOD_bank_bit0_9(4), GPIOD_bit_bit0_9(4), 1);	 // unmute speak
    set_gpio_mode(GPIOD_bank_bit0_9(4), GPIOD_bit_bit0_9(4), GPIO_OUTPUT_MODE);
  }
}

static struct aml_m3_platform_data aml_m3_pdata = {
    .is_hp_pluged = &aml_m3_is_hp_pluged,
};
#endif

#ifdef CONFIG_GOODIX_CAPACITIVE_TOUCHSCREEN
#include <linux/goodix_touch.h>


u8 ts_config_data[] = {
#if 1  // For GT801 EVDD=14V nomal
    0x30,0x13,0x05,0x05,0x28,0x02,0x14,0x14,0x10,0x32,
	0xF2,0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,
	0xAB,0xCD,0xE1,0x00,0x00,0x32,0x32,0x49,0xC1,0x20,
	0x09,0x07,0x07,0x50,0x3C,0x1E,0xB4,0x00,0x33,0x2C,
	0x01,0xEC,0x00,0x46,0x32,0x71,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x01		
#else  // For GT801 EVDD=7.5V
	0x30,0x13,0x05,0x05,0x28,0x02,0x14,0x14,0x10,0x3C,
	0xB2,0x14,0x00,0x1E,0x00,0x01,0x23,0x45,0x67,0x89,
	0xAB,0xCD,0xE1,0x00,0x00,0x32,0x32,0x45,0xCF,0x20,
	0x05,0x05,0x07,0x50,0x3C,0x1E,0xB4,0x00,0x33,0x2C,
	0x01,0xEC,0x00,0x50,0x32,0x71,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x01
#endif
};
   
static struct goodix_i2c_rmi_platform_data ts_pdata = {
    .gpio_shutdown = ((GPIOD_bank_bit0_9(3)<<16) |GPIOD_bit_bit0_9(3)),  // GPIO_D3
    .gpio_irq =  ((GPIOA_bank_bit0_27(16)<<16) | GPIOA_bit_bit0_27(16)), // GPIO_A16
    .irq_edge = 1, /* 0:rising edge, 1:falling edge */
    .swap_xy = 1,
    .xpol = 0,
    .ypol = 1,
    .xmax = 7680,
    .ymax = 5120,
    .config_info_len = ARRAY_SIZE(ts_config_data),
    .config_info = ts_config_data,
};
#endif

#if defined(CONFIG_AML_RTC)
static  struct platform_device aml_rtc_device = {
            .name            = "aml_rtc",
            .id               = -1,
    };
#endif

#if defined (CONFIG_AMLOGIC_VIDEOIN_MANAGER)
static struct resource vm_resources[] = {
    [0] = {
        .start =  VM_ADDR_START,
        .end   = VM_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};
static struct platform_device vm_device =
{
	  .name = "vm",
	  .id = 0,
    .num_resources = ARRAY_SIZE(vm_resources),
    .resource      = vm_resources,
};
#endif /* AMLOGIC_VIDEOIN_MANAGER */

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
static void __init camera_power_on_init(void)
{
    udelay(1000);
    SET_CBUS_REG_MASK(HHI_ETH_CLK_CNTL,0x30f);// 24M XTAL
    SET_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL,0x232);// 24M XTAL
}
#endif

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005)
static void gt2005_v4l2_init(void)
{
    printk(KERN_DEBUG "amlogic camera driver: gt2005_v4l2_init.\n");
    
    // reset active 
    set_gpio_val(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), 0);   
    set_gpio_mode(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), GPIO_OUTPUT_MODE);
	
    // disanable camera power 
    set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 0);    
    set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
	
    msleep(20);
	
	  // reset high
    set_gpio_val(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), 1);    
    set_gpio_mode(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), GPIO_OUTPUT_MODE);
    msleep(20);
    
    // enable camera power 
    set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 1);    
    set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
    msleep(20);	
}
static void gt2005_v4l2_uninit(void)
{
    printk(KERN_DEBUG "amlogic camera driver: gt2005_v4l2_uninit.\n");
    
    set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 0);    
    set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
}
static void gt2005_v4l2_disable(void)
{
    printk(KERN_DEBUG "amlogic camera driver: gt2005_v4l2_disable.\n");
    
    // disable camera power
    set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 0);    
    set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
}

static void gt2005_v4l2_early_suspend(void)
{
    printk(KERN_DEBUG "amlogic camera driver: gt2005_v4l2_early_suspend. \n");
    
    // disable camera power
    //set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 0);    
    //set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
}

static void gt2005_v4l2_late_resume(void)
{
    printk(KERN_DEBUG "amlogic camera driver: gt2005_v4l2_late_resume. \n");
    
    // enable camera power 
    //set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 1);    
    //set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
}

aml_plat_cam_data_t video_gt2005_data = {
    .name="video-gt2005",
    .video_nr=0,   
    .device_init= gt2005_v4l2_init,
	  .device_uninit=gt2005_v4l2_uninit,
	  .early_suspend = gt2005_v4l2_early_suspend,
	  .late_resume = gt2005_v4l2_late_resume,
	  .device_disable=gt2005_v4l2_disable,
};
#endif

#if defined(CONFIG_SUSPEND)
typedef struct {
	  char name[32];
	  unsigned bank;
	  unsigned bit;
	  gpio_mode_t mode;
	  unsigned value;
	  unsigned enable;
} gpio_data_t;

#define MAX_GPIO 0
static gpio_data_t gpio_data[MAX_GPIO] = {
};	

static void save_gpio(int port) 
{
	gpio_data[port].mode = get_gpio_mode(gpio_data[port].bank, gpio_data[port].bit);
	if (gpio_data[port].mode==GPIO_OUTPUT_MODE)
	{
		if (gpio_data[port].enable){
			printk(KERN_DEBUG "change %s output %d to input\n", gpio_data[port].name, gpio_data[port].value); 
			
			gpio_data[port].value = get_gpio_val(gpio_data[port].bank, gpio_data[port].bit);
			set_gpio_mode(gpio_data[port].bank, gpio_data[port].bit, GPIO_INPUT_MODE);
		}
		else{
			printk("no change %s output %d\n", gpio_data[port].name, gpio_data[port].value); 
		}
	}
}

static void restore_gpio(int port)
{
  if ((gpio_data[port].mode==GPIO_OUTPUT_MODE)&&(gpio_data[port].enable))
  {
    printk(KERN_DEBUG "%s output %d\n", gpio_data[port].name, gpio_data[port].value); 
		
		set_gpio_val(gpio_data[port].bank, gpio_data[port].bit, gpio_data[port].value);
		set_gpio_mode(gpio_data[port].bank, gpio_data[port].bit, GPIO_OUTPUT_MODE);	
	}
}

typedef struct {
    char name[32];
    unsigned reg;
    unsigned bits;
    unsigned enable;
} pinmux_data_t;


#define MAX_PINMUX	1

pinmux_data_t pinmux_data[MAX_PINMUX] = {
	{"HDMI", 0, (1<<2)|(1<<1)|(1<<0), 1},
};

static unsigned pinmux_backup[6];

static void save_pinmux(void)
{
	int i;
	for (i=0;i<6;i++)
		pinmux_backup[i] = READ_CBUS_REG(PERIPHS_PIN_MUX_0+i);
	  for (i=0;i<MAX_PINMUX;i++){
		  if (pinmux_data[i].enable){
			  printk("%s %x\n", pinmux_data[i].name, pinmux_data[i].bits);
			  clear_mio_mux(pinmux_data[i].reg, pinmux_data[i].bits);
		  }
	  }
}

static void restore_pinmux(void)
{
	int i;
	for (i=0;i<6;i++) {
		 WRITE_CBUS_REG(PERIPHS_PIN_MUX_0+i, pinmux_backup[i]);
	}
}

static void set_vccx2(int power_on)
{
	int i;
    if (power_on){
			restore_pinmux();
			for (i=0;i<MAX_GPIO;i++) {
				restore_gpio(i);
			}
      printk(KERN_INFO "set_vccx2 power up\n");
      //not usses GPIO but PMU AXP202 to control 
      //set_gpio_mode(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), GPIO_OUTPUT_MODE);
      //set_gpio_val(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), 0);        
    }
    else{
      printk(KERN_INFO "set_vccx2 power down\n");        
      //not usses GPIO but PMU AXP202 to control 
      //set_gpio_mode(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), GPIO_OUTPUT_MODE);
      //set_gpio_val(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), 1);
		  save_pinmux();
		  for (i=0;i<MAX_GPIO;i++){
			  save_gpio(i);
			}
    }
}

static struct meson_pm_config aml_pm_pdata = {
    .pctl_reg_base = IO_APB_BUS_BASE,
    .mmc_reg_base = APB_REG_ADDR(0x1000),
    .hiu_reg_base = CBUS_REG_ADDR(0x1000),
    .power_key = (1<<8),
    .ddr_clk = 0x00110820,
    .sleepcount = 128,
    .set_vccx2 = set_vccx2,
    .core_voltage_adjust = 7,  //5,8
};

static struct platform_device aml_pm_device = {
    .name           = "pm-meson",
    .dev = {
        .platform_data  = &aml_pm_pdata,
    },
    .id             = -1,
};
#endif


#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
static struct aml_i2c_platform aml_i2c_plat = {
    .wait_count     = 50000,
    .wait_ack_interval  = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no      = AML_I2C_MASTER_A,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_pinmux = {
        .scl_reg    = MESON_I2C_MASTER_GPIOX_26_REG,
        .scl_bit    = MESON_I2C_MASTER_GPIOX_26_BIT,
        .sda_reg    = MESON_I2C_MASTER_GPIOX_25_REG,
        .sda_bit    = MESON_I2C_MASTER_GPIOX_25_BIT,
    }
};

static struct aml_i2c_platform aml_i2c_plat1 = {
    .wait_count     = 50000,
    .wait_ack_interval  = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no      = AML_I2C_MASTER_B,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_pinmux = {
        .scl_reg    = MESON_I2C_MASTER_GPIOX_28_REG,
        .scl_bit    = MESON_I2C_MASTER_GPIOX_28_BIT,
        .sda_reg    = MESON_I2C_MASTER_GPIOX_27_REG,
        .sda_bit    = MESON_I2C_MASTER_GPIOX_27_BIT,
    }
};

static struct aml_i2c_platform aml_i2c_plat2 = {
    .wait_count     = 50000,
    .wait_ack_interval  = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no      = AML_I2C_MASTER_AO,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_100K,

    .master_pinmux = {
        .scl_reg    = MESON_I2C_MASTER_GPIOAO_4_REG,
        .scl_bit    = MESON_I2C_MASTER_GPIOAO_4_BIT,
        .sda_reg    = MESON_I2C_MASTER_GPIOAO_5_REG,
        .sda_bit    = MESON_I2C_MASTER_GPIOAO_5_BIT,
    }
};

static struct resource aml_i2c_resource[] = {
	[0]= {
		.start =    MESON_I2C_MASTER_A_START,
		.end   =    MESON_I2C_MASTER_A_END,
		.flags =    IORESOURCE_MEM,
	}
};

static struct resource aml_i2c_resource1[] = {
	[0]= {
		.start =    MESON_I2C_MASTER_A_START,
		.end   =    MESON_I2C_MASTER_A_END,
		.flags =    IORESOURCE_MEM,
  }
};

static struct resource aml_i2c_resource2[] = {
	[0]= {
		.start =    MESON_I2C_MASTER_AO_START,
		.end   =    MESON_I2C_MASTER_AO_END,
		.flags =    IORESOURCE_MEM,
	}
};

static struct platform_device aml_i2c_device = {
    .name         = "aml-i2c",
    .id       = 0,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource),
    .resource     = aml_i2c_resource,
    .dev = {
        .platform_data = &aml_i2c_plat,
    },
};

static struct platform_device aml_i2c_device1 = {
    .name         = "aml-i2c",
    .id       = 1,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource1),
    .resource     = aml_i2c_resource1,
    .dev = {
        .platform_data = &aml_i2c_plat1,
    },
};

static struct platform_device aml_i2c_device2 = {
    .name         = "aml-i2c",
    .id       = 2,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource2),
    .resource     = aml_i2c_resource2,
    .dev = {
        .platform_data = &aml_i2c_plat2,
    },
};

#endif

#ifdef CONFIG_EFUSE
static bool efuse_data_verify(unsigned char *usid)
{  int len;
  
    len = strlen(usid);
    if((len > 8)&&(len<31) )
        return true;
	else
        return false;
}

static struct efuse_platform_data aml_efuse_plat = {
    .pos = 337,
    .count = 30,
    .data_verify = efuse_data_verify,
};

static struct platform_device aml_efuse_device = {
    .name	= "efuse",
    .id	= -1,
    .dev = {
                .platform_data = &aml_efuse_plat,
           },
};
#endif

#ifdef CONFIG_AM_NAND
static struct mtd_partition multi_partition_info[] = 
{
	{
		.name = "logo",
		.offset = 32*SZ_1M+40*SZ_1M,
		.size = 8*SZ_1M,
	},
	{
		.name = "aml_logo",
		.offset = 48*SZ_1M+40*SZ_1M,
		.size = 8*SZ_1M,
	},
	{
		.name = "recovery",
		.offset = 64*SZ_1M+40*SZ_1M,
		.size = 8*SZ_1M,
	},
	{
		.name = "boot",
		.offset = 96*SZ_1M+40*SZ_1M,
		.size = 8*SZ_1M,
	},
	{
		.name = "system",
		.offset = 128*SZ_1M+40*SZ_1M,
		.size = 512*SZ_1M,
	},
	{
		.name = "cache",
		.offset = 640*SZ_1M+40*SZ_1M,
		.size = 128*SZ_1M,
	},
	{
		.name = "userdata",
		.offset = 768*SZ_1M+40*SZ_1M,
		.size = 1024*SZ_1M,
	},
	{
		.name = "NFTL_Part",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
	},
};

static struct aml_nand_platform aml_nand_mid_platform[] = {
{
		.name = NAND_BOOT_NAME,
		.chip_enable_pad = AML_NAND_CE0,
		.ready_busy_pad = AML_NAND_CE0,
		.platform_nand_data = {
			.chip =  {
				.nr_chips = 1,
				.options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE),
			},
    	},
			.T_REA = 20,
			.T_RHOH = 15,
	},
  {
		.name = NAND_MULTI_NAME,
		.chip_enable_pad = (AML_NAND_CE0 | (AML_NAND_CE1 << 4) | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)),
		.ready_busy_pad = (AML_NAND_CE0 | (AML_NAND_CE1 << 4) | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)),
		.platform_nand_data = {
			.chip =  {
				.nr_chips = 2,
				.nr_partitions = ARRAY_SIZE(multi_partition_info),
				.partitions = multi_partition_info,
				.options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE | NAND_TWO_PLANE_MODE),
			},
    	},
			.T_REA = 20,
			.T_RHOH = 15,
	}
};

struct aml_nand_device aml_nand_mid_device = {
	  .aml_nand_platform = aml_nand_mid_platform,
	  .dev_num = ARRAY_SIZE(aml_nand_mid_platform),
};

static struct resource aml_nand_resources[] = {
    {
        .start = 0xc1108600,
        .end = 0xc1108624,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device aml_nand_device = {
    .name = "aml_m3_nand",
    .id = 0,
    .num_resources = ARRAY_SIZE(aml_nand_resources),
    .resource = aml_nand_resources,
    .dev = {
		.platform_data = &aml_nand_mid_device,
    },
};
#endif

#if defined(CONFIG_AMLOGIC_BACKLIGHT)

extern void power_on_backlight(void);
extern void power_off_backlight(void);
extern unsigned get_backlight_level(void);
extern void set_backlight_level(unsigned level);

struct aml_bl_platform_data aml_bl_platform =
{    
    .power_on_bl = power_on_backlight,
    .power_off_bl = power_off_backlight,
    .get_bl_level = get_backlight_level,
    .set_bl_level = set_backlight_level,
};

static struct platform_device aml_bl_device = {
    .name = "aml-bl",
    .id = -1,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &aml_bl_platform,
    },
};
#endif

#if  defined(CONFIG_AM_TV_OUTPUT)||defined(CONFIG_AM_TCON_OUTPUT)
static struct resource vout_device_resources[] = {
    [0] = {
        .start = 0,
        .end   = 0,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vout_device = {
    .name       = "mesonvout",
    .id         = 0,
    .num_resources = ARRAY_SIZE(vout_device_resources),
    .resource      = vout_device_resources,
};
#endif

#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
       .nluns = 2,
       .vendor = "Ainol",
       .product = "NOVO8",
       .release = 0x0100,
};
static struct platform_device usb_mass_storage_device = {
       .name = "usb_mass_storage",
       .id = -1,
       .dev = {
               .platform_data = &mass_storage_pdata,
               },
};
#endif
static char *usb_functions[] = { "usb_mass_storage" };
static char *usb_functions_adb[] = { 
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
"usb_mass_storage", 
#endif

#ifdef CONFIG_USB_ANDROID_ADB
"adb" 
#endif
};
static struct android_usb_product usb_products[] = {
       {
               .product_id     = 0x0c01,
               .num_functions  = ARRAY_SIZE(usb_functions),
               .functions      = usb_functions,
       },
       {
               .product_id     = 0x0c02,
               .num_functions  = ARRAY_SIZE(usb_functions_adb),
               .functions      = usb_functions_adb,
       },
};

static struct android_usb_platform_data android_usb_pdata = {
       .vendor_id      = 0x0bb4,
       .product_id     = 0x0c01,
       .version        = 0x0100,
       .product_name   = "NOVO8",
       .manufacturer_name = "Ainol",
       .num_products = ARRAY_SIZE(usb_products),
       .products = usb_products,
       .num_functions = ARRAY_SIZE(usb_functions_adb),
       .functions = usb_functions_adb,
};

static struct platform_device android_usb_device = {
       .name   = "android_usb",
       .id             = -1,
       .dev            = {
               .platform_data = &android_usb_pdata,
       },
};
#endif

#ifdef CONFIG_POST_PROCESS_MANAGER
static struct resource ppmgr_resources[] = {
    [0] = {
        .start = PPMGR_ADDR_START,
        .end   = PPMGR_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device ppmgr_device = {
    .name       = "ppmgr",
    .id         = 0,
    .num_resources = ARRAY_SIZE(ppmgr_resources),
    .resource      = ppmgr_resources,
};
#endif

#ifdef CONFIG_BT_DEVICE
#include <linux/bt-device.h>

static struct platform_device bt_device = {
	.name             = "bt-dev",
	.id               = -1,
};

static void bt_device_init(void)
{
	/* BT_RST_N */
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<26));  // GPIOD_6
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1, (1<<15));
	
	/* UART_RTS_N(BT) */
	SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_4, (1<<10));
		
	/* UART_CTS_N(BT) */ 
	SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_4, (1<<11));
	
	/* UART_TX(BT) */
	SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_4, (1<<13));
	
	/* UART_RX(BT) */
	SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_4, (1<<12));

  /* BT_WAKE */
  CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO4_EN_N, (1 << 10));
  SET_CBUS_REG_MASK(PREG_PAD_GPIO4_O, (1 << 10));
}

static void bt_device_on(void)
{
  /* reg_on */
  BT_ON = 1;
  reg_on_control(1);
	/* BT_RST_N */
	set_gpio_mode(GPIOD_bank_bit0_9(6), GPIOD_bit_bit0_9(6), GPIO_OUTPUT_MODE);   
	set_gpio_val(GPIOD_bank_bit0_9(6), GPIOD_bit_bit0_9(6), 0);
	msleep(200);	
	set_gpio_val(GPIOD_bank_bit0_9(6), GPIOD_bit_bit0_9(6), 1);
}

static void bt_device_off(void)
{
  BT_ON = 0;
  reg_on_control(0);
	/* BT_RST_N */
	set_gpio_mode(GPIOD_bank_bit0_9(6), GPIOD_bit_bit0_9(6), GPIO_OUTPUT_MODE);
	set_gpio_val(GPIOD_bank_bit0_9(6), GPIOD_bit_bit0_9(6), 0);	
	msleep(200);	
}

static void bt_device_suspend(void)
{
  CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO4_O, (1 << 10));  
}

static void bt_device_resume(void)
{    
  SET_CBUS_REG_MASK(PREG_PAD_GPIO4_O, (1 << 10));
}

struct bt_dev_data bt_dev = {
    .bt_dev_init    = bt_device_init,
    .bt_dev_on      = bt_device_on,
    .bt_dev_off     = bt_device_off,
    .bt_dev_suspend = bt_device_suspend,
    .bt_dev_resume  = bt_device_resume,
};
#endif

#ifdef CONFIG_JOGBALL_INPUT_GPIO
#include <linux/gpio_event.h>
#include <linux/amlogic_jogball.h>
static bool nav_just_on;
static int nav_on_jiffies;
//#define JOGBALL_DEBUG

#define GPIOA4 GPIOA_bank_bit0_27(4)
#define GPIOA4_IRQ					((GPIOA_bank_bit0_27(4) << 16 ) | (GPIOA_bit_bit0_27(4)))
#define GPIOA4_INT INT_GPIO_1

#define GPIOA5 GPIOA_bank_bit0_27(5)
#define GPIOA5_IRQ					((GPIOA_bank_bit0_27(5) << 16 ) | (GPIOA_bit_bit0_27(5)))
#define GPIOA5_INT INT_GPIO_3

#define GPIOA6 GPIOA_bank_bit0_27(6)
#define GPIOA6_IRQ					((GPIOA_bank_bit0_27(6) << 16 ) | (GPIOA_bit_bit0_27(6)))
#define GPIOA6_INT INT_GPIO_6       /* INT_GPIO_4 is used for wifi driver(broadcom 4329) */

#define GPIOA7 GPIOA_bank_bit0_27(7)
#define GPIOA7_IRQ					((GPIOA_bank_bit0_27(7) << 16 ) |(GPIOA_bit_bit0_27(7)))
#define GPIOA7_INT INT_GPIO_5


#define GPIO_JOGBALL_UP_0		(GPIOA4_IRQ)  //GPIOA_4
#define GPIO_JOGBALL_DOWN_0	(GPIOA5_IRQ)  //GPIOA_5
#define GPIO_JOGBALL_LEFT_0	(GPIOA6_IRQ)  //GPIOA_6
#define GPIO_JOGBALL_RIGHT_0 (GPIOA7_IRQ) //GPIOA_7

int gpio_to_irq(unsigned gpio)
{
	/*
	int ret, irq;
	ret = gpio_get_irq_num(gpio, &irq, NULL);
	if (ret)
		return ret;
	return irq;
	*/
	int ret, irq;
	if(gpio==GPIOA4_IRQ)
		ret=GPIOA4_INT;
	else if(gpio==GPIOA5_IRQ)
		ret=GPIOA5_INT;
	else if(gpio==GPIOA6_IRQ)
		ret=GPIOA6_INT;
	else if(gpio==GPIOA7_IRQ)
		ret=GPIOA7_INT;
  
	return ret;
		
}
EXPORT_SYMBOL(gpio_to_irq);

#ifdef JOGBALL_DEBUG
#define cheerchp_debug(...) printk(__VA_ARGS__)
#else
#define cheerchp_debug(...)
#endif

struct amlogic_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
	uint16_t temp_state;
	uint16_t threshold;
	//int in_state;
	//int out_state;
	//int temp_state;
	//int threshold;
};

int amlogic_jogball_irq_init(void)
{
/* memson
	Bit(s)	Description
	256-105	Unused
	104		JTAG_TDO
	103		JTAG_TDI
	102		JTAG_TMS
	101		JTAG_TCK
	100		gpioA_23
	99		gpioA_24
	98		gpioA_25
	97		gpioA_26
	98-75	gpioE[21:0]
	75-50	gpioD[24:0]
	49-23	gpioC[26:0]
	22-15	gpioB[22;15]
	14-0		gpioA[14:0]
 */
  int value;
  printk("jogball irq init(%x,%x,%x,%x)\n", GPIOA4_IRQ, GPIOA5_IRQ, GPIOA6_IRQ, GPIOA7_IRQ);
	value = gpio_get_value(GPIOA4);
	gpio_direction_input(GPIOA4_IRQ);
	gpio_enable_edge_int(gpio_to_idx(GPIOA4_IRQ),value ? 1:0, GPIOA4_INT-INT_GPIO_0);

	value = gpio_get_value(GPIOA5);
	gpio_direction_input(GPIOA5_IRQ);
	gpio_enable_edge_int(gpio_to_idx(GPIOA5_IRQ),value ? 1:0, GPIOA5_INT-INT_GPIO_0);

	value = gpio_get_value(GPIOA6);
	gpio_direction_input(GPIOA6_IRQ);
	gpio_enable_edge_int(gpio_to_idx(GPIOA6_IRQ), value ? 1:0, GPIOA6_INT-INT_GPIO_0);
		
	value = gpio_get_value(GPIOA7);
	gpio_direction_input(GPIOA7_IRQ);
	gpio_enable_edge_int(gpio_to_idx(GPIOA7_IRQ), value ? 1:0, GPIOA7_INT-INT_GPIO_0);

	return 0;
}


uint16_t amlogic_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	
	cheerchp_debug("Charles: Enter amlogic_axis_map!!!!!!\n");
	struct amlogic_axis_info *ai = container_of(info, struct amlogic_axis_info, info);
	uint16_t out = ai->out_state;
	//int out = ai->out_state;

	if (nav_just_on) 
	{
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	
	if((ai->in_state ^ in) & 1)
		out--;
	if((ai->in_state ^ in) & 2)
		out++;
	cheerchp_debug("Charles:ai->out_state=%d\n",ai->out_state);
	ai->out_state = out;
	
ignore:
	ai->in_state = in;
	
	if (ai->out_state - ai->temp_state == ai->threshold) 
	{
		cheerchp_debug("Charles1:info->code=%d\n",info->code);
		/*
		if(info->code==REL_Y)
		{
			ai->temp_state--;
		}
		else
		{
			ai->temp_state++;
		}
		*/
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} 
	else if (ai->temp_state - ai->out_state == ai->threshold) 
	{
		cheerchp_debug("Charles2:info->code=%d\n",info->code);
		/*
		if(info->code==REL_Y)
		{
			ai->temp_state++;
		}
		else
		{
			ai->temp_state--;
		}
		*/
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} 
	else if (abs(ai->out_state - ai->temp_state) > ai->threshold)
	{
		ai->temp_state = ai->out_state;
  }
  
  cheerchp_debug("Charles:ai->temp_state=%d\n",ai->temp_state);
	return ai->temp_state;
	
}

static void sys_led_onoff(int onoff)
{
    if (onoff) {
        set_gpio_mode(GPIOAO_bank_bit0_11(10), GPIOAO_bit_bit0_11(10), GPIO_OUTPUT_MODE);
        set_gpio_val(GPIOAO_bank_bit0_11(10), GPIOAO_bit_bit0_11(10), 1);
    } else {
        set_gpio_mode(GPIOAO_bank_bit0_11(10), GPIOAO_bit_bit0_11(10), GPIO_OUTPUT_MODE);
        set_gpio_val(GPIOAO_bank_bit0_11(10), GPIOAO_bit_bit0_11(10), 0);
	}
}

int amlogic_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
	sys_led_onoff(on);
	return 0;
}

static uint32_t amlogic_x_axis_gpios[] = {
	GPIO_JOGBALL_LEFT_0, GPIO_JOGBALL_RIGHT_0
};

static uint32_t amlogic_x_axis_irq[] = {
	GPIOA6_INT, GPIOA7_INT
};

static struct amlogic_axis_info amlogic_x_axis = {
	.threshold = 1,
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(amlogic_x_axis_gpios),
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(amlogic_x_axis_gpios),
		.map = amlogic_axis_map,
		.gpio = amlogic_x_axis_gpios,
		.irq = amlogic_x_axis_irq,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION | GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT ,
//		.enable_emc_protect_delay = 1 * NSEC_PER_MSEC,
	},
	.in_state=0,
};

static uint32_t amlogic_y_axis_gpios[] = {
	GPIO_JOGBALL_DOWN_0, GPIO_JOGBALL_UP_0
};

static uint32_t amlogic_y_axis_irq[] = {
	GPIOA5_INT, GPIOA4_INT
};

static struct amlogic_axis_info amlogic_y_axis = {
	.threshold = 1,
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(amlogic_y_axis_gpios),
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(amlogic_y_axis_gpios),
		.map = amlogic_axis_map,
		.gpio = amlogic_y_axis_gpios,
		.irq = amlogic_y_axis_irq,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION | GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT  ,
//		.enable_emc_protect_delay = 1 * NSEC_PER_MSEC,
	},
	.in_state=0,
};

static struct gpio_event_info *amlogic_nav_info[] = {
	&amlogic_x_axis.info.info,
	&amlogic_y_axis.info.info,
};

static struct gpio_event_platform_data amlogic_nav_data = {
	//.init_irq=&amlogic_jogball_irq_init,
	.name = "cheerchip-nav",
	.info = amlogic_nav_info,
	.info_count = ARRAY_SIZE(amlogic_nav_info),
	.power = amlogic_nav_power,
};

static struct platform_device amlogic_nav_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.num_resources = 0,
  .resource = NULL,
	.dev		= {
		.platform_data	= &amlogic_nav_data,
	},
};
#endif

static struct platform_device __initdata *platform_devs[] = {
#if defined(CONFIG_FB_AM)
    &fb_device,
#endif
#if defined(CONFIG_AM_STREAMING)
    &codec_device,
#endif
#if defined(CONFIG_AM_DEINTERLACE) || defined (CONFIG_DEINTERLACE)
    &deinterlace_device,
#endif
#if defined(CONFIG_TVIN_VDIN)
    &vdin_device,
#endif
#if defined(CONFIG_TVIN_BT656IN)
	  &bt656in_device,
#endif
#if defined(CONFIG_AML_AUDIO_DSP)
    &audiodsp_device,
#endif
#if defined(CONFIG_SND_AML_M3)
    &aml_audio,
#endif
#if defined(CONFIG_CARDREADER)
    &amlogic_card_device,
#endif
#if defined(CONFIG_AML_RTC)
    &aml_rtc_device,
#endif
#if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_VIRTUAL_REMOTE)||defined(CONFIG_KEYPADS_AM_MODULE)
    &input_device,
#endif
#ifdef CONFIG_SARADC_AM
    &saradc_device,
#endif
#ifdef CONFIG_ADC_TOUCHSCREEN_AM
    &adc_ts_device,
#endif
#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
    &adc_kp_device,
#endif
#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
    &input_device_key,  //changed by Elvis
#endif
#ifdef CONFIG_AM_NAND
    &aml_nand_device,
#endif
#if defined(CONFIG_NAND_FLASH_DRIVER_MULTIPLANE_CE)
    &aml_nand_device,
#endif
#ifdef CONFIG_AMLOGIC_VIDEOIN_MANAGER
	  &vm_device,
#endif
#if defined(CONFIG_SUSPEND)
    &aml_pm_device,
#endif

#if defined(CONFIG_I2C_AML)|| defined(CONFIG_I2C_HW_AML)
    &aml_i2c_device,
    &aml_i2c_device1,
    &aml_i2c_device2,
#endif

#if defined(CONFIG_AMLOGIC_BACKLIGHT)
    &aml_bl_device,
#endif
#if defined(CONFIG_AM_TV_OUTPUT)||defined(CONFIG_AM_TCON_OUTPUT)
    &vout_device,   
#endif
#ifdef CONFIG_USB_ANDROID
    &android_usb_device,
    #ifdef CONFIG_USB_ANDROID_MASS_STORAGE
        &usb_mass_storage_device,
    #endif
#endif
#ifdef CONFIG_BT_DEVICE  
    &bt_device,
#endif

#ifdef CONFIG_EFUSE
	  &aml_efuse_device,
#endif
#ifdef CONFIG_POST_PROCESS_MANAGER
    &ppmgr_device,
#endif
#if defined(CONFIG_USB_PHY_CONTROL)
    &usb_phy_control_device,
#endif
#ifdef CONFIG_ANDROID_TIMED_GPIO
	  &amlogic_gpio_vibravor,
#endif
#ifdef CONFIG_JOGBALL_INPUT_GPIO
  &amlogic_nav_device,
#endif

};

static struct i2c_board_info __initdata aml_i2c_bus_info[] = {
#ifdef CONFIG_GOODIX_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO(GOODIX_I2C_NAME, GOODIX_I2C_ADDR),
        .irq = INT_GPIO_0,
        .platform_data = (void *)&ts_pdata,
    },
#endif

#if CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005
    {
    	/*gt2005 i2c address is 0x78/0x79*/
    	I2C_BOARD_INFO("gt2005_i2c",  0x78 >> 1 ),
    	.platform_data = (void *)&video_gt2005_data
    },
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_OV2655
	{
		I2C_BOARD_INFO("ov2655_i2c", 0x60 >> 1),
		.platform_data = (void *)&video_ov2655_data,
	},
#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info_1[] = {
#ifdef CONFIG_BOSCH_BMA222
	{
		I2C_BOARD_INFO("bma222",  0x18),
		//.irq = INT_GPIO_1,
	},
#endif

#ifdef CONFIG_SENSORS_MMC328X
	{
		I2C_BOARD_INFO(MMC328X_I2C_NAME,  MMC328X_I2C_ADDR),
	},
#endif

#ifdef CONFIG_SENSORS_MMA8452
	{
		I2C_BOARD_INFO(MMA8452_I2C_NAME,  MMA8452_I2C_ADDR),
	},
#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info_2[] = {
#ifdef CONFIG_AW_AXP20
	{
    I2C_BOARD_INFO("axp20_mfd", AXP20_ADDR),
		.platform_data = &axp_pdata,
		//.irq = pmu_irq_id,
	},
#endif

};

static int __init aml_i2c_init(void)
{
#ifdef CONFIG_AW_AXP20
    pmu_twi_id = AXP20_I2CBUS;
    pmu_irq_id = AXP20_IRQNO;
    pmu_twi_addr = AXP20_ADDR;
    pmu_battery_rdc = 70;//BATRDC;
    pmu_battery_cap = 7800;//BATTERYCAP;
    pmu_init_chgcur = INTCHGCUR ;
    pmu_suspend_chgcur = SUSCHGCUR ;
    pmu_resume_chgcur = RESCHGCUR ;
    pmu_shutdown_chgcur = CLSCHGCUR ;       
    pmu_init_chgvol = INTCHGVOL ;
    pmu_init_chgend_rate = INTCHGENDRATE;
    pmu_init_chg_enabled = INTCHGENABLED;
    pmu_init_adc_freq = INTADCFREQ;
    pmu_init_adc_freq = INTADCFREQC;
    pmu_init_chg_pretime = INTCHGPRETIME;
    pmu_init_chg_csttime = INTCHGCSTTIME;
    pmu_bat_para1 = 0;//OCVREG0;
    pmu_bat_para2 = 0;//OCVREG1;
    pmu_bat_para3 = 1;//OCVREG2;   
    pmu_bat_para4 = 2;//OCVREG3;   
    pmu_bat_para5 = 3;//OCVREG4;   
    pmu_bat_para6 = 4;//OCVREG5;
    pmu_bat_para7 = 5;//OCVREG6;
    pmu_bat_para8 = 9;//OCVREG7;
    pmu_bat_para9 = 14;//OCVREG8;
    pmu_bat_para10 = 20;//OCVREG9;   
    pmu_bat_para11 = 28;//OCVREGA;  
    pmu_bat_para12 = 43;//OCVREGB;
    pmu_bat_para13 = 76;//OCVREGC;    
    pmu_bat_para14 = 78;//OCVREGD;  
    pmu_bat_para15 = 82;//OCVREGE;    
    pmu_bat_para16 = 100;//OCVREGF;   
    pmu_usbvol_limit = 1;    
    pmu_usbvol = 4000;   
    pmu_usbcur_limit = 0; 
    pmu_usbcur = 900;
    pmu_pwroff_vol = 2600; 
    pmu_pwron_vol = 2600;    
    dcdc2_vol = 1520;//1400;   
    dcdc3_vol = 1220;//1250;    
    ldo2_vol = 3000;    
    ldo3_vol = 2800;       
    ldo4_vol = 2800;       
    pmu_pekoff_time = 6000;       
    pmu_pekoff_en   = 1;       
    pmu_peklong_time = 1500;       
    pmu_pwrok_time    = 64;      
    pmu_pwrnoe_time = 2000;       
    pmu_intotp_en = 1;       
    pmu_pekon_time = 1000;
                  
    //axp_regl_init_data[1].constraints.state_standby.uV = ldo2_vol * 1000;
    //axp_regl_init_data[2].constraints.state_standby.uV = ldo3_vol * 1000;
    //axp_regl_init_data[3].constraints.state_standby.uV = ldo4_vol * 1000;
    axp_regl_init_data[5].constraints.state_standby.uV = dcdc2_vol * 1000;
    axp_regl_init_data[6].constraints.state_standby.uV = dcdc3_vol * 1000;
    battery_data.voltage_max_design = pmu_init_chgvol;
    battery_data.voltage_min_design = pmu_pwroff_vol * 1000;
    battery_data.energy_full_design = pmu_battery_cap;
    axp_sply_init_data.chgcur = pmu_init_chgcur;
    axp_sply_init_data.chgvol = pmu_init_chgvol;
    axp_sply_init_data.chgend = pmu_init_chgend_rate;
    axp_sply_init_data.chgen = pmu_init_chg_enabled;
    axp_sply_init_data.sample_time = pmu_init_adc_freq;
    axp_sply_init_data.chgpretime = pmu_init_chg_pretime;
    axp_sply_init_data.chgcsttime = pmu_init_chg_csttime;
    aml_i2c_bus_info_2[0].addr = pmu_twi_addr;
    aml_i2c_bus_info_2[0].irq = pmu_irq_id;

#endif
    i2c_register_board_info(0, aml_i2c_bus_info,
        ARRAY_SIZE(aml_i2c_bus_info));
    i2c_register_board_info(1, aml_i2c_bus_info_1,
        ARRAY_SIZE(aml_i2c_bus_info_1)); 
    i2c_register_board_info(2, aml_i2c_bus_info_2,
        ARRAY_SIZE(aml_i2c_bus_info_2)); 
    return 0;
}

#if defined(CONFIG_TVIN_BT656IN)
static void __init bt656in_pinmux_init(void)
{
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, 0xf<<6);
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_3, 1<<21);
}
#endif

static void __init device_pinmux_init(void )
{
    clearall_pinmux();
    aml_i2c_init();
#if defined(CONFIG_TVIN_BT656IN)
    bt656in_pinmux_init();
#endif
    // set_audio_pinmux(AUDIO_OUT_TEST_N);
    // set_audio_pinmux(AUDIO_IN_JTAG);
    WRITE_CBUS_REG(HHI_GEN_CLK_CNTL,(READ_CBUS_REG(HHI_GEN_CLK_CNTL)&(~(0x7f<<0)))|((0<<0)|(1<<8)|(7<<9)) );
    CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<15));    
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_3, (1<<22));

    // set clk for camera 
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1, (1<<29)); 
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<2));

    unsigned pwm_cnt = get_ddr_pll_clk()/48000000 - 1;
    pwm_cnt &= 0xffff;
    WRITE_CBUS_REG(PWM_PWM_C, (pwm_cnt<<16) | pwm_cnt);
    SET_CBUS_REG_MASK(PWM_MISC_REG_CD, (1<<15)|(0<<8)|(1<<4)|(1<<0)); //select ddr pll for source, and clk divide 
}

static void disable_unused_model(void)
{
    CLK_GATE_OFF(VIDEO_IN);
    CLK_GATE_OFF(BT656_IN);
    CLK_GATE_OFF(ETHERNET);
    //CLK_GATE_OFF(SATA);
    //CLK_GATE_OFF(WIFI);
    video_dac_disable();
 }
static void __init power_hold(void)
{
    printk(KERN_INFO "power hold set high!\n");
    set_gpio_val(GPIOAO_bank_bit0_11(6), GPIOAO_bit_bit0_11(6), 1);
    set_gpio_mode(GPIOAO_bank_bit0_11(6), GPIOAO_bit_bit0_11(6), GPIO_OUTPUT_MODE);
    
    //VCCx2 power up
    printk(KERN_INFO "set_vccx2 power up\n");
    set_gpio_mode(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), GPIO_OUTPUT_MODE);
    set_gpio_val(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), 0);
}

static void __init LED_PWM_REG0_init(void)
{
        // Enable VBG_EN
    WRITE_CBUS_REG_BITS(PREG_AM_ANALOG_ADDR, 1, 0, 1);
    // wire pm_gpioA_7_led_pwm = pin_mux_reg0[22];
    WRITE_CBUS_REG(LED_PWM_REG0,(0 << 31)   |       // disable the overall circuit
                                (0 << 30)   |       // 1:Closed Loop  0:Open Loop
                                (0 << 16)   |       // PWM total count
                                (0 << 13)   |       // Enable
                                (1 << 12)   |       // enable
                                (0 << 10)   |       // test
                                (7 << 7)    |       // CS0 REF, Voltage FeedBack: about 0.505V
                                (7 << 4)    |       // CS1 REF, Current FeedBack: about 0.505V
                                READ_CBUS_REG(LED_PWM_REG0)&0x0f);           // DIMCTL Analog dimmer
                                
    WRITE_CBUS_REG_BITS(LED_PWM_REG0,1,0,4); //adust cpu1.2v   to 1.26V     

}

static __init void m3_init_machine(void)
{
    meson_cache_init();
#ifdef CONFIG_AML_SUSPEND
    extern int (*pm_power_suspend)(void);
    pm_power_suspend = meson_power_suspend;
#endif /*CONFIG_AML_SUSPEND*/

#if defined(CONFIG_AMLOGIC_BACKLIGHT)
	power_off_backlight();
#endif
    LED_PWM_REG0_init();
    power_hold();
    // uses PMU to control.
    // pm_power_off = power_off;		//Elvis fool
    device_pinmux_init();
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
    camera_power_on_init();
#endif
    platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

#ifdef CONFIG_USB_DWC_OTG_HCD
    set_usb_phy_clk(USB_PHY_CLOCK_SEL_XTAL_DIV2);
    lm_device_register(&usb_ld_a);
    set_usb_phy_id_mode(USB_PHY_PORT_B,USB_PHY_MODE_SW_HOST);
    lm_device_register(&usb_ld_b);
#endif
#ifdef CONFIG_SATA_DWC_AHCI
    set_sata_phy_clk(SATA_PHY_CLOCK_SEL_DEMOD_PLL);
    lm_device_register(&sata_ld);
#endif
    disable_unused_model();
    // hp detect
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0,1,19,1);
#ifdef CONFIG_JOGBALL_INPUT_GPIO    
    amlogic_jogball_irq_init();
    sys_led_onoff(1);
#endif
}

/*VIDEO MEMORY MAPING*/
static __initdata struct map_desc meson_video_mem_desc[] = {
    {
        .virtual    = PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
        .pfn        = __phys_to_pfn(RESERVED_MEM_START),
        .length     = RESERVED_MEM_END-RESERVED_MEM_START+1,
        .type       = MT_DEVICE,
    },
#ifdef CONFIG_AML_SUSPEND
    {
        .virtual    = PAGE_ALIGN(__phys_to_virt(PHYS_OFFSET + CONFIG_AML_SUSPEND_FIRMWARE_BASE)),
        .pfn        = __phys_to_pfn(PHYS_OFFSET + CONFIG_AML_SUSPEND_FIRMWARE_BASE),
        .length     = SZ_1M,
        .type       = MT_MEMORY,
    },
#endif
};

static __init void m3_map_io(void)
{
    meson_map_io();
    iotable_init(meson_video_mem_desc, ARRAY_SIZE(meson_video_mem_desc));
}

static __init void m3_irq_init(void)
{
    meson_init_irq();
}

static __init void m3_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
    struct membank *pbank;
    m->nr_banks = 0;
    pbank=&m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(PHYS_MEM_START);
    pbank->size  = SZ_64M & PAGE_MASK;
    pbank->node  = PHYS_TO_NID(PHYS_MEM_START);
    m->nr_banks++;
    // RESERVED_MEM_END ~ PHYS_MEM_END 
    pbank=&m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(RESERVED_MEM_END+1);
#ifdef CONFIG_AML_SUSPEND
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END-SZ_1M) & PAGE_MASK;
#else
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END) & PAGE_MASK;
#endif
    pbank->node  = PHYS_TO_NID(RESERVED_MEM_END+1);
    m->nr_banks++;
}

MACHINE_START(MESON3_8726M_SKT, "AMLOGIC MESON3 8726M SKT SH")
    .phys_io        = MESON_PERIPHS1_PHYS_BASE,
    .io_pg_offst    = (MESON_PERIPHS1_PHYS_BASE >> 18) & 0xfffc,
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = m3_map_io,
    .init_irq       = m3_irq_init,
    .timer          = &meson_sys_timer,
    .init_machine   = m3_init_machine,
    .fixup          = m3_fixup,
    .video_start    = RESERVED_MEM_START,
    .video_end      = RESERVED_MEM_END,
MACHINE_END
