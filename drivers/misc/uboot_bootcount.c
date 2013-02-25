/*
 * This driver gives access(read/write) to the bootcounter used by u-boot.
 * Access is supported via sysFS.
 *
 * Copyright 2008 DENX Software Engineering GmbH
 * Author: Heiko Schocher <hs@denx.de>
 * Based on work from: Steffen Rumler  <Steffen.Rumler@siemens.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>

#define UBOOT_BOOTCOUNT_NAME "bootcount"

#define	UBOOT_BOOTCOUNT_MAGIC		0xB001C041 /* magic number value */

static void __iomem *mem;
static int single_word;
static u32 magic_offset;
static u32 magic_mask;
static u32 value_mask;

/* helper for the sysFS */
static int show_str_bootcount(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long counter;

	counter = readl(mem) & value_mask;

	return sprintf(buf, "%lu\n", counter);
}

static int store_str_bootcount(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int r;
	u32 value;
	unsigned long magic;

	magic = readl(mem + magic_offset);
	if ((magic & magic_mask) != (UBOOT_BOOTCOUNT_MAGIC & magic_mask))
		return -EINVAL;

	r = kstrtou32(buf, 0, &value);
	if (r < 0)
		return -EINVAL;

	if (single_word)
		writel((UBOOT_BOOTCOUNT_MAGIC & magic_mask) | value, mem);
	else
		writel(value, mem);

	return count;
}

static DEVICE_ATTR(bootcount, S_IWUSR | S_IRUGO, show_str_bootcount,
		store_str_bootcount);

static const struct file_operations bootcount_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice bootcount_miscdev = {
	MISC_DYNAMIC_MINOR,
	UBOOT_BOOTCOUNT_NAME,
	&bootcount_fops
};

static int bootcount_probe(struct platform_device *ofdev)
{
	struct device_node *np = of_node_get(ofdev->dev.of_node);
	unsigned long magic;
	struct resource res;
	int ret;

	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		return -ENODEV;

	/*
	 * Auto-detect if single-word (4-bytes) or double-word (8-bytes)
	 * bootcounter is used and save the macros for later usage
	 */
	if (resource_size(&res) == 4) {
		single_word = 1;
		magic_offset = 0;
		magic_mask = 0xffff0000;
		value_mask = 0x0000ffff;
	} else {
		single_word = 0;
		magic_offset = 4;
		magic_mask = 0xffffffff;
		value_mask = 0xffffffff;
	}

	mem = ioremap(res.start, resource_size(&res));
	if (mem == NULL) {
		dev_err(&ofdev->dev, "couldnt map register.\n");
		return -ENODEV;
	}

	magic = readl(mem + magic_offset);
	if ((magic & magic_mask) != (UBOOT_BOOTCOUNT_MAGIC & magic_mask)) {
		dev_err(&ofdev->dev, "bad magic!\n");
		goto no_magic;
	}

	if (misc_register(&bootcount_miscdev)) {
		dev_err(&ofdev->dev, "failed to register device\n");
		goto misc_register_fail;
	}

	if (device_create_file(bootcount_miscdev.this_device,
		&dev_attr_bootcount)) {
		dev_warn(&ofdev->dev, "couldnt register sysFS entry.\n");
		goto register_sysfs_fail;
	}

	dev_info(&ofdev->dev,
		 "U-Boot bootcounter driver registered using %s-word\n",
		 single_word ? "single" : "double");

	return 0;

register_sysfs_fail:
	misc_deregister(&bootcount_miscdev);
misc_register_fail:
no_magic:
	iounmap(mem);
	return -ENODEV;
}

static int bootcount_remove(struct platform_device *ofdev)
{
	misc_deregister(&bootcount_miscdev);
	iounmap(mem);

	return 0;
}

static const struct of_device_id bootcount_match[] = {
	{
		.compatible = "uboot,bootcount",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bootcount_match);

static struct platform_driver bootcount_driver = {
	.driver = {
		.name = UBOOT_BOOTCOUNT_NAME,
		.of_match_table = bootcount_match,
		.owner = THIS_MODULE,
	},
	.probe = bootcount_probe,
	.remove = bootcount_remove,
};

module_platform_driver(bootcount_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steffen Rumler <steffen.rumler@siemens.com>");
MODULE_DESCRIPTION("Provide (read/write) access to the U-Boot bootcounter via sysFS");
