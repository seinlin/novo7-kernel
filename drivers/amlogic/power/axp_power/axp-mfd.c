/*
 * Base driver for AXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <mach/am_regs.h>
#include <mach/gpio.h>

#include "axp18-mfd.h"
#include "axp19-mfd.h"
#include "axp20-mfd.h"

struct axp_cfg_info *axp_cfg_board;

static inline int is_ac_online(void)
{
	uint8_t val;
	axp_read(&axp->dev,0x00, &val);
	if(val & ((1<<7) | (1<<5)))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static void axp_mfd_irq_work(struct work_struct *work)
{
	struct axp_mfd_chip *chip =
		container_of(work, struct axp_mfd_chip, irq_work);
	uint64_t irqs = 0;

	printk("[AXP]========================in irq=====================\n");
	
	while (1) {
		if (chip->ops->read_irqs(chip, &irqs))
			break;

		printk("%s->%d:irqs = 0x%x\n",__FUNCTION__,__LINE__,(int) irqs);
		
		irqs &= chip->irqs_enabled;
		
		printk("%s->%d: chip->irqs_enabled = 0x%x\n",__FUNCTION__,__LINE__,(int) chip->irqs_enabled);
		
		if (irqs == 0)
			break;

		printk("%s->%d:irqs = 0x%x\n",__FUNCTION__,__LINE__,(int) irqs);

		blocking_notifier_call_chain(
				&chip->notifier_list, irqs, NULL);
	}
	enable_irq(chip->client->irq);
}

#if 1
static irqreturn_t axp_mfd_irq_handler(int irq, void *data)
{
	struct axp_mfd_chip *chip = data;
	disable_irq_nosync(irq);
	(void)schedule_work(&chip->irq_work);

	return IRQ_HANDLED;
}
#endif

static struct axp_mfd_chip_ops axp_mfd_ops[] = {
	[0] = {
		.init_chip    = axp18_init_chip,
		.enable_irqs  = axp18_enable_irqs,
		.disable_irqs = axp18_disable_irqs,
		.read_irqs    = axp18_read_irqs,
	},
	[1] = {
		.init_chip    = axp19_init_chip,
		.enable_irqs  = axp19_enable_irqs,
		.disable_irqs = axp19_disable_irqs,
		.read_irqs    = axp19_read_irqs,
	},
	[2] = {
		.init_chip    = axp20_init_chip,
		.enable_irqs  = axp20_enable_irqs,
		.disable_irqs = axp20_disable_irqs,
		.read_irqs    = axp20_read_irqs,
	},
};

static const struct i2c_device_id axp_mfd_id_table[] = {
	{ "axp18_mfd", 0 },
	{ "axp19_mfd", 1 },
	{ "axp20_mfd", 2 },
	{},
};
MODULE_DEVICE_TABLE(i2c, axp_mfd_id_table);

int axp_mfd_create_attrs(struct axp_mfd_chip *chip)
{
	int j,ret;
	if(chip->type ==  AXP19){
		for (j = 0; j < ARRAY_SIZE(axp19_mfd_attrs); j++) {
			ret = device_create_file(chip->dev,&axp19_mfd_attrs[j]);
			if (ret)
				goto sysfs_failed;
		}
	}
	else if (chip->type ==  AXP18){
		for (j = 0; j < ARRAY_SIZE(axp18_mfd_attrs); j++) {
			ret = device_create_file(chip->dev,&axp18_mfd_attrs[j]);
			if (ret)
			goto sysfs_failed2;
		}
	}
	else if (chip->type ==  AXP20){
		for (j = 0; j < ARRAY_SIZE(axp20_mfd_attrs); j++) {
			ret = device_create_file(chip->dev,&axp20_mfd_attrs[j]);
			if (ret)
			goto sysfs_failed3;
		}
	}
	else
		ret = 0;
	goto succeed;

sysfs_failed:
	while (j--)
		device_remove_file(chip->dev,&axp19_mfd_attrs[j]);
	goto succeed;
sysfs_failed2:
	while (j--)
		device_remove_file(chip->dev,&axp18_mfd_attrs[j]);
	goto succeed;
sysfs_failed3:
	while (j--)
		device_remove_file(chip->dev,&axp20_mfd_attrs[j]);
succeed:
	return ret;
}

static int __remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int axp_mfd_remove_subdevs(struct axp_mfd_chip *chip)
{
	return device_for_each_child(chip->dev, NULL, __remove_subdev);
}

static int __devinit axp_mfd_add_subdevs(struct axp_mfd_chip *chip,
					struct axp_platform_data *pdata)
{
	struct axp_funcdev_info *regl_dev;
	struct axp_funcdev_info *sply_dev;
	struct axp_funcdev_info *gpio_dev;
	struct platform_device *pdev;
	int i, ret = 0;

	/* register for regultors */
	for (i = 0; i < pdata->num_regl_devs; i++) {
		regl_dev = &pdata->regl_devs[i];
		pdev = platform_device_alloc(regl_dev->name, regl_dev->id);
		pdev->dev.parent = chip->dev;
		pdev->dev.platform_data = regl_dev->platform_data;
		ret = platform_device_add(pdev);
		if (ret)
			goto failed;
	}

	/* register for power supply */
	for (i = 0; i < pdata->num_sply_devs; i++) {
	sply_dev = &pdata->sply_devs[i];
	pdev = platform_device_alloc(sply_dev->name, sply_dev->id);
	pdev->dev.parent = chip->dev;
	pdev->dev.platform_data = sply_dev->platform_data;
	ret = platform_device_add(pdev);
	if (ret)
		goto failed;

	}

	/* register for gpio */
	for (i = 0; i < pdata->num_gpio_devs; i++) {
	gpio_dev = &pdata->gpio_devs[i];
	pdev = platform_device_alloc(gpio_dev->name, gpio_dev->id);
	pdev->dev.parent = chip->dev;
	pdev->dev.platform_data = gpio_dev->platform_data;
	ret = platform_device_add(pdev);
	if (ret)
		goto failed;
	}


	return 0;

failed:
	axp_mfd_remove_subdevs(chip);
	return ret;
}

 void axp_power_off(void)
{
    printk("[axp] send power-off command!\n");
    mdelay(20);
	axp_set_bits(&axp->dev, POWER20_OFF_CTL, 0x80);
    mdelay(20);
    printk("[axp] warning!!! axp can't power-off, maybe some error happend!\n");
}
EXPORT_SYMBOL_GPL(axp_power_off);

static int __devinit axp_mfd_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct axp_platform_data *pdata = client->dev.platform_data;
	struct axp_mfd_chip *chip;
	int ret;
	chip = kzalloc(sizeof(struct axp_mfd_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	axp = client;
	axp_cfg_board = pdata->axp_cfg;

	chip->client = client;
	chip->dev = &client->dev;
	chip->ops = &axp_mfd_ops[id->driver_data];

	mutex_init(&chip->lock);
	INIT_WORK(&chip->irq_work, axp_mfd_irq_work);
	BLOCKING_INIT_NOTIFIER_HEAD(&chip->notifier_list);

	i2c_set_clientdata(client, chip);

	ret = chip->ops->init_chip(chip);
	if (ret)
		goto out_free_chip;
    /*
	ret = request_irq(client->irq, axp_mfd_irq_handler,
		IRQF_DISABLED, "axp_mfd", chip);
  	if (ret) {
  		dev_err(&client->dev, "failed to request irq %d\n",
  				client->irq);
  		goto out_free_chip;
  	}
    */

	ret = axp_mfd_add_subdevs(chip, pdata);
	if (ret)
		goto out_free_irq;

	/* PM hookup */
	if(!pm_power_off)
		pm_power_off = axp_power_off;

	ret = axp_mfd_create_attrs(chip);
	if(ret){
		return ret;
	}

	return 0;

out_free_irq:
	free_irq(client->irq, chip);

out_free_chip:
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return ret;
}

static int __devexit axp_mfd_remove(struct i2c_client *client)
{
	struct axp_mfd_chip *chip = i2c_get_clientdata(client);

	pm_power_off = NULL;
	axp = NULL;

	axp_mfd_remove_subdevs(chip);
	kfree(chip);
	return 0;
}

static struct i2c_driver axp_mfd_driver = {
	.driver	= {
		.name	= "axp_mfd",
		.owner	= THIS_MODULE,
	},
	.probe		= axp_mfd_probe,
	.remove		= __devexit_p(axp_mfd_remove),
	.id_table	= axp_mfd_id_table,
};

static int __init axp_mfd_init(void)
{
	return i2c_add_driver(&axp_mfd_driver);
}
arch_initcall(axp_mfd_init);

static void __exit axp_mfd_exit(void)
{
	i2c_del_driver(&axp_mfd_driver);
}
module_exit(axp_mfd_exit);

MODULE_DESCRIPTION("PMIC MFD Driver for AXP");
MODULE_AUTHOR("Donglu Zhang Krosspower");
MODULE_LICENSE("GPL");
