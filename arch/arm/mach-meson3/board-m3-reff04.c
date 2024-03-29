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
#include "board-m3-reff04.h"


#ifdef CONFIG_SENSORS_MMC328X
#include <linux/mmc328x.h>
#endif

#ifdef CONFIG_SENSORS_MMA8452
#include <linux/mma8452.h>
#endif

#ifdef CONFIG_SIX_AXIS_SENSOR_MPU3050
#ifdef CONFIG_MPU_PRE_V340
#include <linux/mpu.h>
#else
#include <linux/mpu_new/mpu.h>
#endif
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

#ifdef CONFIG_BQ27x00_BATTERY
#include <linux/bq27x00_battery.h>
#endif


#ifdef CONFIG_EFUSE
#include <linux/efuse.h>
#endif



#if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_KEYPADS_AM_MODULE)
static struct resource intput_resources[] = {
    {
        .start = 0x0,
        .end = 0x0,
        .name="8726M3",
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
    {KEY_BACK,          "exit", CHAN_4, 386, 60},
    {KEY_HOME,          "home", CHAN_4, 508, 60},
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


#ifdef CONFIG_SIX_AXIS_SENSOR_MPU3050
#define GPIO_mpu3050_PENIRQ ((GPIOA_bank_bit0_27(14)<<16) | GPIOA_bit_bit0_27(14))
#define MPU3050_IRQ  INT_GPIO_1
static int mpu3050_init_irq(void)
{
    /* set input mode */
    gpio_direction_input(GPIO_mpu3050_PENIRQ);
    /* map GPIO_mpu3050_PENIRQ map to gpio interrupt, and triggered by rising edge(=0) */
    gpio_enable_edge_int(gpio_to_idx(GPIO_mpu3050_PENIRQ), 0, MPU3050_IRQ-INT_GPIO_0);
    return 0;
}

static struct mpu3050_platform_data mpu3050_data = {
    .int_config = 0x10,
    .orientation = {0,-1,0,-1,0,0,0,0,-1}, 
    .level_shifter = 0,
    .accel = {
                .get_slave_descr = get_accel_slave_descr,
                .adapt_num = 1, // The i2c bus to which the mpu device is
                // connected
                .bus = EXT_SLAVE_BUS_SECONDARY, //The secondary I2C of MPU
                .address = 0x1c,
                .orientation = {0,1,0,1,0,0,0,0,-1},  
            },
    #ifdef CONFIG_SENSORS_MMC314X
    .compass = {
                .get_slave_descr = mmc314x_get_slave_descr,
                .adapt_num = 0, // The i2c bus to which the compass device is. 
                // It can be difference with mpu
                // connected
                .bus = EXT_SLAVE_BUS_PRIMARY,
                .address = 0x30,
                .orientation = { -1, 0, 0,  0, 1, 0,  0, 0, -1 },
           } 
#endif

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
static void set_usb_a_vbus_power(char is_power_on)
{
#define USB_A_POW_GPIO          GPIOD_bank_bit0_9(9)
#define USB_A_POW_GPIO_BIT      GPIOD_bit_bit0_9(9)
#define USB_A_POW_GPIO_BIT_ON   1
#define USB_A_POW_GPIO_BIT_OFF  0
    if(is_power_on) {
        printk(KERN_INFO "set usb port power on (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        set_gpio_mode(USB_A_POW_GPIO, USB_A_POW_GPIO_BIT, GPIO_OUTPUT_MODE);
        set_gpio_val(USB_A_POW_GPIO, USB_A_POW_GPIO_BIT, USB_A_POW_GPIO_BIT_ON);
    } else    {
        printk(KERN_INFO "set usb port power off (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        set_gpio_mode(USB_A_POW_GPIO, USB_A_POW_GPIO_BIT, GPIO_OUTPUT_MODE);
        set_gpio_val(USB_A_POW_GPIO, USB_A_POW_GPIO_BIT, USB_A_POW_GPIO_BIT_OFF);
    }
}
//usb_a is OTG port
static struct lm_device usb_ld_a = {
    .type = LM_DEVICE_TYPE_USB,
    .id = 0,
    .irq = INT_USB_A,
    .resource.start = IO_USB_A_BASE,
    .resource.end = -1,
    .dma_mask_room = DMA_BIT_MASK(32),
    .port_type = USB_PORT_TYPE_OTG,
    .port_speed = USB_PORT_SPEED_DEFAULT,
    .dma_config = USB_DMA_BURST_SINGLE,
    .set_vbus_power = set_usb_a_vbus_power,
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
//add pin mux info for bt656 input
#if 0
static struct resource bt656in_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,      //pbufAddr
        .end   = VDIN_ADDR_END,             //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {     //bt656/camera/bt601 input resource pin mux setting
        .start =  0x3000,       //mask--mux gpioD 15 to bt656 clk;  mux gpioD 16:23 to be bt656 dt_in
        .end   = PERIPHS_PIN_MUX_5 + 0x3000,
        .flags = IORESOURCE_MEM,
    },

    [2] = {         //camera/bt601 input resource pin mux setting
        .start =  0x1c000,      //mask--mux gpioD 12 to bt601 FIQ; mux gpioD 13 to bt601HS; mux gpioD 14 to bt601 VS;
        .end   = PERIPHS_PIN_MUX_5 + 0x1c000,
        .flags = IORESOURCE_MEM,
    },

    [3] = {         //bt601 input resource pin mux setting
        .start =  0x800,        //mask--mux gpioD 24 to bt601 IDQ;;
        .end   = PERIPHS_PIN_MUX_5 + 0x800,
        .flags = IORESOURCE_MEM,
    },

};
#endif

static struct platform_device bt656in_device = {
    .name       = "amvdec_656in",
    .id         = -1,
//    .num_resources = ARRAY_SIZE(bt656in_resources),
//    .resource      = bt656in_resources,
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

void extern_wifi_power(int is_power)
{
/*
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1,(1<<11));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0,(1<<18));
	CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<8));
	if(is_power)
		SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
	else
		CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));  
*/
}

void sdio_extern_init(void)
{
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
        .card_power_en_reg = EGPIO_GPIOC_ENABLE,
        .card_power_en_mask = PREG_IO_7_MASK,
        .card_power_output_reg = EGPIO_GPIOC_OUTPUT,
        .card_power_output_mask = PREG_IO_7_MASK,
        .card_power_en_lev = 1,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = sdio_extern_init,
    },
};

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

extern char* get_vout_mode_internal(void);

/* Check current mode, 0: panel; 1: !panel*/
int get_display_mode(void) {
	int ret = 0;
	if(strncmp("panel", get_vout_mode_internal(), 5))
		ret = 1;
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
#ifdef _AML_M3_HW_DEBUG_
  printk("***Entered %s:%s\n", __FILE__,__func__);
#endif
  if(flag){
    set_gpio_val(GPIOC_bank_bit0_15(4), GPIOC_bit_bit0_15(4), 0);    // mute speak
    set_gpio_mode(GPIOC_bank_bit0_15(4), GPIOC_bit_bit0_15(4), GPIO_OUTPUT_MODE);
  }else{
    set_gpio_val(GPIOC_bank_bit0_15(4), GPIOC_bit_bit0_15(4), 1);    // unmute speak
    set_gpio_mode(GPIOC_bank_bit0_15(4), GPIOC_bit_bit0_15(4), GPIO_OUTPUT_MODE);
  }
}


static struct aml_m3_platform_data aml_m3_pdata = {
    .is_hp_pluged = &aml_m3_is_hp_pluged,
};
#endif

#ifdef CONFIG_ITK_CAPACITIVE_TOUCHSCREEN
#include <linux/i2c/itk.h>
#define GPIO_ITK_PENIRQ ((GPIOA_bank_bit0_27(16)<<16) | GPIOA_bit_bit0_27(16))
#define GPIO_ITK_RST ((GPIOC_bank_bit0_15(3)<<16) | GPIOC_bit_bit0_15(3))
#define ITK_INT INT_GPIO_0
static int itk_init_irq(void)
{
    /* set input mode */
    gpio_direction_input(GPIO_ITK_PENIRQ);
    /* set gpio interrupt #0 source=GPIOD_24, and triggered by falling edge(=1) */
    gpio_enable_edge_int(gpio_to_idx(GPIO_ITK_PENIRQ), 1, ITK_INT-INT_GPIO_0);

    return 0;
}
static int itk_get_irq_level(void)
{
    return gpio_get_value(GPIO_ITK_PENIRQ);
}

void touch_on(int flag)
{
	printk("enter %s flag=%d \n",__FUNCTION__,flag);
	if(flag)
		gpio_direction_output(GPIO_ITK_RST, 1);
	else
		gpio_direction_output(GPIO_ITK_RST, 0);
}

static struct itk_platform_data itk_pdata = {
    .init_irq = &itk_init_irq,
    .get_irq_level = &itk_get_irq_level,
    .touch_on =  touch_on,
    .tp_max_width = 3328,
    .tp_max_height = 2432,
    .lcd_max_width = 800,
    .lcd_max_height = 600,
};
#endif

#ifdef CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN
#include <linux/i2c/pixcir_i2c_ts.h>
static struct pixcir_i2c_ts_platform_data pixcir_pdata = {
	.gpio_shutdown = (GPIOC_bank_bit0_15(3)<<16) | GPIOC_bit_bit0_15(3), //GPIO_C3
	.gpio_irq = (GPIOA_bank_bit0_27(16)<<16) | GPIOA_bit_bit0_27(16), //GPIO_A16
	.xmin = 0,
	.xmax = 1025,
	.ymin = 0,
	.ymax = 600,
  .swap_xy = 0,
  .xpol = 0,
  .ypol = 0,
  .point_id_available = 0,	
};
#endif

#ifdef CONFIG_FOCALTECH_CAPACITIVE_TOUCHSCREEN
#include <linux/ft5x06_ts.h>
#define TS_IRQ_GPIO  ((GPIOA_bank_bit0_27(16)<<16) | GPIOA_bit_bit0_27(16))

static int ts_init_irq(void);
static int ts_get_irq_level(void);

static struct ts_platform_data ts_pdata = {
    .mode = TS_MODE_INT_FALLING,
    .irq = INT_GPIO_0,
    .init_irq = ts_init_irq,
    .get_irq_level = ts_get_irq_level,
    .info = {
        .xmin = 0,
        .xmax = 4095,
        .ymin = 0,
        .ymax = 4095,
        .zmin = 0,
        .zmax = 1,
        .wmin = 0,
        .wmax = 1,
        .swap_xy = 0,
        .x_pol = 1,
        .y_pol = 1
    },
    .data = 0,
};
static int ts_init_irq(void)
{
    int group = ts_pdata.irq - INT_GPIO_0;
    int mode =  ts_pdata.mode;

    if (mode < TS_MODE_TIMER_READ) {
        gpio_direction_input(TS_IRQ_GPIO);
        if (mode == TS_MODE_INT_FALLING) {
            gpio_enable_edge_int(gpio_to_idx(TS_IRQ_GPIO), 1, group);
        }
        else if (mode == TS_MODE_INT_RISING) {
            gpio_enable_edge_int(gpio_to_idx(TS_IRQ_GPIO), 0, group);
        }
        else if (mode == TS_MODE_INT_LOW) {
            gpio_enable_level_int(gpio_to_idx(TS_IRQ_GPIO), 1, group);
        }
        else if (mode == TS_MODE_INT_HIGH) {
            gpio_enable_level_int(gpio_to_idx(TS_IRQ_GPIO), 0, group);
        }
    }
    return 0;
}

static int ts_get_irq_level(void)
{
    return gpio_get_value(TS_IRQ_GPIO);
}
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_data =
{
    .name = "pmem",
    .start = PMEM_START,
    .size = PMEM_SIZE,
    .no_allocator = 1,
    .cached = 0,
};

static struct platform_device android_pmem_device =
{
    .name = "android_pmem",
    .id = 0,
    .dev = {
        .platform_data = &pmem_data,
    },
};
#endif

#if defined(CONFIG_AML_RTC)
static  struct platform_device aml_rtc_device = {
            .name            = "aml_rtc",
            .id               = -1,
    };
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308
int gc0308_init(void)
{
//    udelay(1000);
//    WRITE_CBUS_REG(HHI_ETH_CLK_CNTL,0x30f);// 24M XTAL
//    WRITE_CBUS_REG(HHI_DEMOD_PLL_CNTL,0x232);// 24M XTAL
//	udelay(1000);
	
    // set camera power disable
    set_gpio_val(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), 0);    // set camera power disable
    set_gpio_mode(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), GPIO_OUTPUT_MODE);
    msleep(20);

    set_gpio_val(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), 1);    // set camera power disable
    set_gpio_mode(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), GPIO_OUTPUT_MODE);
    msleep(20);
    
    // set camera power enable
    set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 0);    // set camera power enable
    set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
    msleep(20);
    return 0;
}
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

    //eth_set_pinmux(ETH_BANK0_GPIOC3_C12,ETH_CLK_OUT_GPIOC12_REG3_1, 1);		
}
#endif
#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308)
static void gc0308_v4l2_init(void)
{
	printk( "amlogic camera driver: gc0308_v4l2_init. \n");
	gc0308_init();
}
static void gc0308_v4l2_uninit(void)
{

	printk( "amlogic camera driver: gc0308_v4l2_uninit. \n");
    set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 1);    // set camera power disable
    set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
}
static void gc0308_v4l2_early_suspend(void)
{
    set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 1);    // set camera power disable
    set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
}

static void gc0308_v4l2_late_resume(void)
{
    set_gpio_val(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), 0);    // set camera power enable
    set_gpio_mode(GPIOA_bank_bit0_27(25), GPIOA_bit_bit0_27(25), GPIO_OUTPUT_MODE);
}

struct aml_camera_i2c_fig1_s custom_gc0308_script[] = {  
	{0x14,0x11}, // h_v
	{0x26,0x01},
	{0xff,0xff}, 
};


aml_plat_cam_data_t video_gc0308_data = {
	.name="video-gc0308",
	.video_nr=1,//1,
	.device_init= gc0308_v4l2_init,
	.device_uninit=gc0308_v4l2_uninit,
	.custom_init_script = custom_gc0308_script,
	.early_suspend = gc0308_v4l2_early_suspend,
	.late_resume = gc0308_v4l2_late_resume,
};


#endif


#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005)
static void gt2005_v4l2_init(void)
{
    // reset low 
    printk( "amlogic camera driver: gt2005_v4l2_init. \n");
    set_gpio_val(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), 0);   
    set_gpio_mode(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), GPIO_OUTPUT_MODE);
	
    // set camera power disanable
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 0);    
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
	
    msleep(20);
	
	// reset high
    set_gpio_val(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), 1);    
    set_gpio_mode(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), GPIO_OUTPUT_MODE);
    msleep(20);
    
    // set camera power enable
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 1);    
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
    msleep(20);
}
static void gt2005_v4l2_uninit(void)
{
	printk( "amlogic camera driver: gt2005_v4l2_uninit.\n");
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 0);    // set camera power disable
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
}
static void gt2005_v4l2_disable(void)
{

}

static void gt2005_v4l2_early_suspend(void)
{
	printk( "amlogic camera driver: gt2005_v4l2_early_suspend. \n");
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 0);    // set camera power disable
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
}

static void gt2005_v4l2_late_resume(void)
{
	printk( "amlogic camera driver: gt2005_v4l2_late_resume. \n");
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 1);    // set camera power enable
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
}

aml_plat_cam_data_t video_gt2005_data = {
	.name="video-gt2005",
	.video_nr=0,   //    1
	.device_init= gt2005_v4l2_init,
	.device_uninit=gt2005_v4l2_uninit,
	.early_suspend = gt2005_v4l2_early_suspend,
	.late_resume = gt2005_v4l2_late_resume,
	.device_disable=gt2005_v4l2_disable,
};
#endif /* VIDEO_AMLOGIC_CAPTURE_GT2005 */

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_HY253)
static void hy253_v4l2_init(void)
{
	printk( "hy253_v4l2_init. \n");
    // reset low 
    printk( "amlogic camera driver: gt2005_v4l2_init. \n");
    set_gpio_val(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), 0);   
    set_gpio_mode(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), GPIO_OUTPUT_MODE);
	
    // set camera power disanable
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 0);    
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
	
    msleep(20);
	
	// reset high
    set_gpio_val(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), 1);    
    set_gpio_mode(GPIOY_bank_bit0_22(10), GPIOY_bit_bit0_22(10), GPIO_OUTPUT_MODE);
    msleep(20);
    
    // set camera power enable
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 1);    
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
    msleep(20);

}
static void hy253_v4l2_uninit(void)
{
	printk( "hy253_v4l2_uninit. \n");
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 1);    // set camera power disable
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
}
static void hy253_v4l2_disable(void)
{

}

static void hy253_v4l2_early_suspend(void)
{
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 1);    // set camera power disable
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
}

static void hy253_v4l2_late_resume(void)
{
    set_gpio_val(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), 0);    // set camera power enable
    set_gpio_mode(GPIOA_bank_bit0_27(24), GPIOA_bit_bit0_27(24), GPIO_OUTPUT_MODE);
}

aml_plat_cam_data_t video_hy253_data = {
	.name="video-hy253",
	.video_nr=1,
	.device_init= hy253_v4l2_init,
	.device_uninit=hy253_v4l2_uninit,
	.early_suspend = hy253_v4l2_early_suspend,
	.late_resume = hy253_v4l2_late_resume,
	.device_disable=hy253_v4l2_disable,
};

#endif /* CONFIG_VIDEO_AMLOGIC_CAPTURE_HY253 */

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
			printk("change %s output %d to input\n", gpio_data[port].name, gpio_data[port].value); 
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
		set_gpio_val(gpio_data[port].bank, gpio_data[port].bit, gpio_data[port].value);
		set_gpio_mode(gpio_data[port].bank, gpio_data[port].bit, GPIO_OUTPUT_MODE);
		// printk("%s output %d\n", gpio_data[port].name, gpio_data[port].value); 
	}
}

typedef struct {
	char name[32];
	unsigned reg;
	unsigned bits;
	unsigned enable;
} pinmux_data_t;


#define MAX_PINMUX	0

pinmux_data_t pinmux_data[MAX_PINMUX] = {
	//{"HDMI", 	0, (1<<2)|(1<<1)|(1<<0), 						1},
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
	for (i=0;i<6;i++)
		 WRITE_CBUS_REG(PERIPHS_PIN_MUX_0+i, pinmux_backup[i]);
}

static void set_vccx2(int power_on)
{
	int i;
    if (power_on){

		restore_pinmux();
		for (i=0;i<MAX_GPIO;i++)
			restore_gpio(i);
        printk(KERN_INFO "set_vccx2 power up\n");
        set_gpio_mode(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), GPIO_OUTPUT_MODE);
        set_gpio_val(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), 0);
              
    }
    else{
        printk(KERN_INFO "set_vccx2 power down\n");        
        set_gpio_mode(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), GPIO_OUTPUT_MODE);
        set_gpio_val(GPIOA_bank_bit0_27(26), GPIOA_bit_bit0_27(26), 1);
		save_pinmux();
		for (i=0;i<MAX_GPIO;i++)
			save_gpio(i);
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

#ifdef CONFIG_PMU_ACT8xxx
#include <linux/act8xxx.h>  

/*
 *	DC_DET(GPIOA_20)	enable internal pullup
 *		High:		Disconnect
 *		Low:		Connect
 */
static inline int is_ac_online(void)
{
	int val;
	
	SET_CBUS_REG_MASK(PAD_PULL_UP_REG0, (1<<20));	//enable internal pullup
	set_gpio_mode(GPIOA_bank_bit0_27(20), GPIOA_bit_bit0_27(20), GPIO_INPUT_MODE);
	val = get_gpio_val(GPIOA_bank_bit0_27(20), GPIOA_bit_bit0_27(20));
	
	logd("%s: get from gpio is %d.\n", __FUNCTION__, val);
	
	return !val;
}


static void power_off(void)
{
    if(is_ac_online()){ //AC in after power off press
        kernel_restart("charging_reboot");
    }
    
    //BL_PWM power off
    set_gpio_val(GPIOD_bank_bit0_9(1), GPIOD_bit_bit0_9(1), 0);
    set_gpio_mode(GPIOD_bank_bit0_9(1), GPIOD_bit_bit0_9(1), GPIO_OUTPUT_MODE);

    //VCCx2 power down
    set_vccx2(0);
    
    //Power hold down
    set_gpio_val(GPIOAO_bank_bit0_11(6), GPIOAO_bit_bit0_11(6), 0);
    set_gpio_mode(GPIOAO_bank_bit0_11(6), GPIOAO_bit_bit0_11(6), GPIO_OUTPUT_MODE);
}

/*
//temporary
static inline int is_usb_online(void)
{
	u8 val;

	return 0;
}
*/

#ifdef CONFIG_PMU_ACT8942
/*
 *	Fast charge when CHG_CON(GPIOAO_11) is High.
 *	Slow charge when CHG_CON(GPIOAO_11) is Low.
 */
static int set_charge_current(int level)
{
	set_gpio_mode(GPIOAO_bank_bit0_11(11), GPIOAO_bit_bit0_11(11), GPIO_OUTPUT_MODE);
	set_gpio_val(GPIOAO_bank_bit0_11(11), GPIOAO_bit_bit0_11(11), (level ? 1 : 0));
	return 0;
}


/*
 *	nSTAT OUTPUT(GPIOA_21)	enable internal pullup
 *		High:		Full
 *		Low:		Charging
 */
static inline int get_charge_status(void)
{
	int val;
	
	SET_CBUS_REG_MASK(PAD_PULL_UP_REG0, (1<<21));	//enable internal pullup
	set_gpio_mode(GPIOA_bank_bit0_27(21), GPIOA_bit_bit0_27(21), GPIO_INPUT_MODE);
	val = get_gpio_val(GPIOA_bank_bit0_27(21), GPIOA_bit_bit0_27(21));

	logd("%s: get from gpio is %d.\n", __FUNCTION__, val);
	return val;
}

static inline int get_bat_adc_value(void)
{
	return get_adc_sample(5);
}


/*
 *	VBAT_DET CHAN5 Vbat=Vadc*2
 */
static inline int _measure_current(void)
{
	int val, Vgnd, Vbatn, vdiff;
	set_gpio_mode(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), GPIO_OUTPUT_MODE);
	set_gpio_val(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), 1);
	msleep(5);
	Vbatn = get_adc_sample(7) * (2500000 / 1023);
	
	set_gpio_mode(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), GPIO_OUTPUT_MODE);
	set_gpio_val(GPIOA_bank_bit0_27(22), GPIOA_bit_bit0_27(22), 0);
	msleep(5);
	Vgnd = get_adc_sample(7) * (2500000 / 1023);
	
	vdiff = Vbatn - Vgnd;
	val = (vdiff*1047)/(110*2);
	
	//printk("%s Vbatn:%duV Vgnd:%duV vdiff:%duV I:%duA.\n", (vdiff>0)?"charging...":"uncharging...", Vbatn, Vgnd, vdiff, val);
	return val;
	
}

static inline int _measure_voltage(void)
{
	int val, Vbat, Icur;
	
retry:
	val = get_bat_adc_value();
	if(val == -1){
	    goto retry;
	}
	
	val =  val * (2 * 2500000 / 1023);
	
	Icur = _measure_current();
	Vbat = val - Icur*102/1000;  // 0.102
	//printk("v:%d vbat:%d\n", val, Vbat);
	return Vbat;
}
 
static inline int measure_voltage(void)
{
	return _measure_voltage();
}

static inline int measure_current(void)
{
	return _measure_current();
}

static int bat_level_table[37]={
0,
0,
4,
10,
15,
16,
18,
20,
23,
26,
29,
32,
35,
37,
40,
43,
46,
49,
51,
54,
57,
60,
63,
66,
68,
71,
74,
77,
80,
83,
85,
88,
91,
95,
97,
100,
100  
};

static int new_bat_value_table[37]={
0,  //0    
3500000,//0  
3518899,//4  
3549407,//10 
3573985,//15 
3591305,//16 
3632629,//18 
3655652,//20 
3665652,//23 
3688675,//26 
3717997,//29 
3733844,//32 
3742432,//35   
3759835,//37 
3768423,//40 
3778423,//43 
3789157,//46 
3803818,//49 
3817293,//51 
3830768,//54 
3851502,//57 
3869864,//60 
3888226,//63 
3911475,//66  
3928282,//68  
3938282,//71  
3944272,//74  
3954272,//77  
3966705,//80  
3986705,//83  
4003142,//85  
4036165,//88  
4060600,//91  
4083849,//95  
4093849,//97  
4108389,//100 
4200000 //100
};

static inline int get_bat_percentage(int adc_vaule, int *adc_table, 
										int *per_table, int table_size)
{
	int i;
	for(i=0; i<(table_size - 1); i++) {
		if ((adc_vaule >= adc_table[i]) && (adc_vaule < adc_table[i+1])) 
			break;
	}
  //printk("per_table[%d]=%d\n",i, per_table[i]);
	return per_table[i];
}

static int act8942_measure_capacity_charging(void)
{
    int vbat = _measure_voltage()-33000;  // - 33mV
    int table_size = ARRAY_SIZE(new_bat_value_table);

	return get_bat_percentage(vbat, new_bat_value_table, bat_level_table, table_size);
}

static int act8942_measure_capacity_battery(void)
{
    int vbat = _measure_voltage();
	int table_size = ARRAY_SIZE(new_bat_value_table);

	return get_bat_percentage(vbat, new_bat_value_table, bat_level_table, table_size);
}

//temporary
static int set_bat_off(void)
{
	return 0;
}


static struct act8942_operations act8942_pdata = {
	.is_ac_online = is_ac_online,
	//.is_usb_online = is_usb_online,
	.set_bat_off = set_bat_off,
	.get_charge_status = get_charge_status,
	.set_charge_current = set_charge_current,
	.measure_voltage = measure_voltage,
	.measure_current = measure_current,
	.measure_capacity_charging = act8942_measure_capacity_charging,
	.measure_capacity_battery = act8942_measure_capacity_battery,
	.update_period = 2000,	//2S
	.asn = 10,				//Average Sample Number
	.rvp = 1,				//reverse voltage protection: 1:enable; 0:disable
};
#endif

static struct platform_device aml_pmu_device = {
    .name	= ACT8xxx_DEVICE_NAME,
    .id	= -1,
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
		.size = 512*SZ_1M,
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
		.ready_busy_pad = (AML_NAND_CE0 | (AML_NAND_CE0 << 4) | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)),
		.platform_nand_data = {
			.chip =  {
				.nr_chips = 4,
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

#if  defined(CONFIG_AM_TV_OUTPUT2)
static struct resource vout2_device_resources[] = {
    [0] = {
        .start = 0,
        .end   = 0,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vout2_device = {
    .name       = "mesonvout2",
    .id         = 0,
    .num_resources = ARRAY_SIZE(vout2_device_resources),
    .resource      = vout2_device_resources,
};
#endif

#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
       .nluns = 2,
       .vendor = "Ramos",
       .product = "W6HD",
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
       .product_name   = "W6HD",
       .manufacturer_name = "Ramos",
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
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<16));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1, (1<<5));
	
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
	/* BT_RST_N */
	CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<6));
	CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<6));	
	msleep(200);	
	SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<6));
}

static void bt_device_off(void)
{
	/* BT_RST_N */
	CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<6));
	CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<6));	
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
#if defined(CONFIG_AM_TV_OUTPUT2)
    &vout2_device,   
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
#ifdef CONFIG_PMU_ACT8xxx
	&aml_pmu_device,
#endif
#ifdef CONFIG_POST_PROCESS_MANAGER
    &ppmgr_device,
#endif
#if defined(CONFIG_USB_PHY_CONTROL)
    &usb_phy_control_device,
#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info[] = {

#ifdef CONFIG_ITK_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("itk", 0x41),
        .irq = ITK_INT,
        .platform_data = (void *)&itk_pdata,
    },
#endif

#ifdef CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("pixcir168", 0x5c),
        .irq = INT_GPIO_0,
        .platform_data = (void *)&pixcir_pdata,
    },
#endif

#ifdef CONFIG_FOCALTECH_CAPACITIVE_TOUCHSCREEN	//Added by Leon 20110920
    {
        I2C_BOARD_INFO("ft5x06", 0x38),
        .irq = INT_GPIO_0,
        .platform_data = (void *)&ts_pdata,
    },
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308

	{
        /*gc0308 i2c address is 0x42/0x43*/
		I2C_BOARD_INFO("gc0308_i2c",  0x42 >> 1),
		.platform_data = (void *)&video_gc0308_data,
	},
#endif

#if CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005
    {
    	/*gt2005 i2c address is 0x78/0x79*/
    	I2C_BOARD_INFO("gt2005_i2c",  0x78 >> 1 ),
    	.platform_data = (void *)&video_gt2005_data
    },
#endif

#if CONFIG_VIDEO_AMLOGIC_CAPTURE_HY253
    {
    	/*hy253 i2c address is 0x78/0x79*/
    	I2C_BOARD_INFO("hy253_i2c",  0x40 >> 1 ),
    	.platform_data = (void *)&video_hy253_data
    },
#endif

//#ifdef CONFIG_CAMERA_OV9650FSL
//#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info_1[] = {
#ifdef CONFIG_SENSORS_MMC328X
	{
		I2C_BOARD_INFO(MMC328X_I2C_NAME,  MMC328X_I2C_ADDR),
	},
#endif
#ifdef CONFIG_SIX_AXIS_SENSOR_MPU3050
    {
        I2C_BOARD_INFO("mpu3050", 0x68),
        .irq = MPU3050_IRQ,
        .platform_data = (void *)&mpu3050_data,
    },
#endif
#ifdef CONFIG_SENSORS_MMA8452
	{
		I2C_BOARD_INFO(MMA8452_I2C_NAME,  MMA8452_I2C_ADDR),
	},
#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info_2[] = {
#ifdef CONFIG_BQ27x00_BATTERY
    {
        I2C_BOARD_INFO("bq27200", 0x55),
        .platform_data = (void *)&bq27x00_pdata,
    },
#endif
#ifdef CONFIG_PMU_ACT8xxx
	{
        I2C_BOARD_INFO(ACT8xxx_I2C_NAME, ACT8xxx_ADDR),
#ifdef CONFIG_PMU_ACT8942
		.platform_data = (void *)&act8942_pdata,
#endif
    },
#endif
};

static int __init aml_i2c_init(void)
{
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
static void __init eth_pinmux_init(void)
{

    CLEAR_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, 1);
    SET_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, (1 << 1));
    SET_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, 1);
    udelay(100);
    aml_i2c_init();
}

static void __init device_pinmux_init(void )
{
    clearall_pinmux();
    aml_i2c_init();
#if defined(CONFIG_TVIN_BT656IN)
    bt656in_pinmux_init();
#endif
    set_audio_pinmux(AUDIO_OUT_TEST_N);
   // set_audio_pinmux(AUDIO_IN_JTAG);
#ifdef CONFIG_SIX_AXIS_SENSOR_MPU3050
    mpu3050_init_irq();
#endif    
#if 1
    //set clk for wifi
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
#endif
}


static void disable_unused_model(void)
{
    CLK_GATE_OFF(VIDEO_IN);
    CLK_GATE_OFF(BT656_IN);
    CLK_GATE_OFF(ETHERNET);
//    CLK_GATE_OFF(SATA);
//    CLK_GATE_OFF(WIFI);
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

/* usb wifi power 1:power on  0:power off */
void extern_usb_wifi_power(int is_power)
{
    printk(KERN_INFO "usb_wifi_power %s\n", is_power ? "On" : "Off");	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1,(1<<11));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0,(1<<18));
	CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<8));
	if(is_power)
		CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));
	else
		SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<8));   
}

EXPORT_SYMBOL(extern_usb_wifi_power);

#if defined(CONFIG_AML_INIT_GATE_OFF)
#define GATE_INIT_OFF(_MOD) CLEAR_CBUS_REG_MASK(GCLK_REG_##_MOD, GCLK_MASK_##_MOD);

static __init void init_gate_off(void) 
{
	//turn of video gates
	GATE_INIT_OFF(VCLK2_VENCP1);
	GATE_INIT_OFF(VCLK2_VENCP);
	GATE_INIT_OFF(VCLK2_VENCL);
	GATE_INIT_OFF(VCLK2_ENCL);
	GATE_INIT_OFF(VCLK2_OTHER1);
	GATE_INIT_OFF(VCLK2_VENCI1);
	GATE_INIT_OFF(VCLK2_VENCI);
	GATE_INIT_OFF(VENC_P_TOP);
	GATE_INIT_OFF(VENC_L_TOP);
	GATE_INIT_OFF(VENC_I_TOP);
	//GATE_INIT_OFF(VCLK2_VENCT);
	//GATE_INIT_OFF(VCLK2_ENCT);
	GATE_INIT_OFF(VENCP_INT);
	GATE_INIT_OFF(VENCL_INT);
	GATE_INIT_OFF(VCLK2_ENCI);
	GATE_INIT_OFF(VCLK2_ENCP);
	GATE_INIT_OFF(VCLK2_OTHER);
	GATE_INIT_OFF(ENC480P);
	GATE_INIT_OFF(VENC_DAC);
	GATE_INIT_OFF(DAC_CLK);
}
#endif    

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
    //pm_power_off = power_off;		//Elvis fool
    device_pinmux_init();
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
    camera_power_on_init();
#endif
	extern_usb_wifi_power(0);
    
#if defined(CONFIG_AML_INIT_GATE_OFF)
    init_gate_off();
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

#ifdef CONFIG_PMU_ACT8942
	set_charge_current(0);   
#endif
    disable_unused_model();
    // hp detect
    WRITE_CBUS_REG_BITS(PAD_PULL_UP_REG0,1,19,1);
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
