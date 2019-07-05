// SPDX-License-Identifier: GPL-2.0-only
/*
 * Mediatek MT7530 DSA Switch driver
 * Copyright (C) 2017 Sean Wang <sean.wang@mediatek.com>
 */
#include <linux/bitfield.h>
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/gpio/consumer.h>
#include <net/dsa.h>
#include <linux/platform_device.h> // test-only

// test-only: for sysc_read/write()
#include <asm/mach-ralink/ralink_regs.h>
#include <asm/mach-ralink/mt7620.h>


/* struct mt7628_priv -	This is the main data structure for holding the state
 *			of the driver
 * @dev:		The device pointer
 * @ds:			The pointer to the dsa core structure
 * @bus:		The bus used for the device and built-in PHY
 * @rstc:		The pointer to reset control used by MCM
 * @ethernet:		The regmap used for access TRGMII-based registers
 * @core_pwr:		The power supplied into the core
 * @io_pwr:		The power supplied into the I/O
 * @reset:		The descriptor for GPIO line tied to its reset pin
 * @mcm:		Flag for distinguishing if standalone IC or module
 *			coupling
 * @ports:		Holding the state among ports
 * @reg_mutex:		The lock for protecting among process accessing
 *			registers
 */
struct mt7628_priv {
	struct device		*dev;
	struct dsa_switch	*ds;
	struct mii_bus		*bus;
	struct reset_control	*rstc;
#if 0
	struct regmap		*ethernet;
	struct regulator	*core_pwr;
	struct regulator	*io_pwr;
	struct gpio_desc	*reset;
#endif
	unsigned int		id;
	bool			mcm;

//	struct mt7628_port	ports[MT7628_NUM_PORTS];
	/* protect among processes for registers access*/
	struct mutex reg_mutex;
};

static int mt7628_setup(struct dsa_switch *ds)
{
	printk("%s (%d)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", __func__, __LINE__); // test-only

	return 0;
}

/* System controller register */
#define MT7628_RSTCTRL_REG	0x34
#define RSTCTRL_EPHY_RST	BIT(24)

#define MT7628_AGPIO_CFG_REG	0x3c
#define MT7628_EPHY_GPIO_AIO_EN	GENMASK(20, 17)
#define MT7628_EPHY_P0_DIS	BIT(16)

#define MT7628_GPIO2_MODE_REG	0x64

/* Ethernet switch register */
#define MT7628_SWITCH_FCT0	0x0008
#define MT7628_SWITCH_PFC1	0x0014
#define MT7628_SWITCH_FPA	0x0084
#define MT7628_SWITCH_SOCPC	0x008c
#define MT7628_SWITCH_POC0	0x0090
#define MT7628_SWITCH_POC2	0x0098
#define MT7628_SWITCH_SGC	0x009c
#define MT7628_SWITCH_PCR0	0x00c0
#define PCR0_PHY_ADDR		GENMASK(4, 0)
#define PCR0_PHY_REG		GENMASK(12, 8)
#define PCR0_WT_PHY_CMD		BIT(13)
#define PCR0_RD_PHY_CMD		BIT(14)
#define PCR0_WT_DATA		GENMASK(31, 16)

#define MT7628_SWITCH_PCR1	0x00c4
#define PCR1_WT_DONE		BIT(0)
#define PCR1_RD_RDY		BIT(1)
#define PCR1_RD_DATA		GENMASK(31, 16)

#define MT7628_SWITCH_FPA1	0x00c8
#define MT7628_SWITCH_FCT2	0x00cc
#define MT7628_SWITCH_SGC2	0x00e4
#define MT7628_SWITCH_BMU_CTRL	0x0110

#define CONFIG_MDIO_TIMEOUT	100
#define CONFIG_DMA_STOP_TIMEOUT	100
#define CONFIG_TX_DMA_TIMEOUT	100

#define LINK_DELAY_TIME		500		/* 500 ms */
#define LINK_TIMEOUT		10000		/* 10 seconds */

// test-only: rename
struct mt7628_esw_priv {
	void __iomem *base;		/* switch base address */

	struct dsa_switch	*ds;

	struct mii_dev *bus;
};

static int mdio_wait_read(struct mt7628_esw_priv *priv, u32 mask, bool mask_set)
{
#if 0
	void __iomem *base = priv->base;
	int ret;

	ret = wait_for_bit_le32(base + MT7628_SWITCH_PCR1, mask, mask_set,
				CONFIG_MDIO_TIMEOUT, false);
	if (ret) {
		printf("MDIO operation timeout!\n");
		return -ETIMEDOUT;
	}
#else
	mdelay(10);
#endif

	return 0;
}

static int mii_mgr_read(struct mt7628_esw_priv *priv,
			u32 phy_addr, u32 phy_register, u32 *read_data)
{
	void __iomem *base = priv->base;
	u32 status = 0;
	u32 ret;

	*read_data = 0xffff;
	/* Make sure previous read operation is complete */
	ret = mdio_wait_read(priv, PCR1_RD_RDY, false);
	if (ret)
		return ret;

	writel(PCR0_RD_PHY_CMD |
	       FIELD_PREP(PCR0_PHY_REG, phy_register) |
	       FIELD_PREP(PCR0_PHY_ADDR, phy_addr),
	       base + MT7628_SWITCH_PCR0);

	/* Make sure previous read operation is complete */
	ret = mdio_wait_read(priv, PCR1_RD_RDY, true);
	if (ret)
		return ret;

	status = readl(base + MT7628_SWITCH_PCR1);
	*read_data = FIELD_GET(PCR1_RD_DATA, status);

	return 0;
}

static int mii_mgr_write(struct mt7628_esw_priv *priv,
			 u32 phy_addr, u32 phy_register, u32 write_data)
{
	void __iomem *base = priv->base;
	u32 data;
	int ret;

	/* Make sure previous write operation is complete */
	ret = mdio_wait_read(priv, PCR1_WT_DONE, false);
	if (ret)
		return ret;

	data = FIELD_PREP(PCR0_WT_DATA, write_data) |
		FIELD_PREP(PCR0_PHY_REG, phy_register) |
		FIELD_PREP(PCR0_PHY_ADDR, phy_addr) |
		PCR0_WT_PHY_CMD;
	writel(data, base + MT7628_SWITCH_PCR0);

	return mdio_wait_read(priv, PCR1_WT_DONE, true);
}

static void mt7628_ephy_init(struct mt7628_esw_priv *priv)
{
	int i;

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	mii_mgr_write(priv, 0, 31, 0x2000);	/* change G2 page */
	mii_mgr_write(priv, 0, 26, 0x0000);

	for (i = 0; i < 5; i++) {
		mii_mgr_write(priv, i, 31, 0x8000);	/* change L0 page */
		mii_mgr_write(priv, i,  0, 0x3100);

		/* EEE disable */
		mii_mgr_write(priv, i, 30, 0xa000);
		mii_mgr_write(priv, i, 31, 0xa000);	/* change L2 page */
		mii_mgr_write(priv, i, 16, 0x0606);
		mii_mgr_write(priv, i, 23, 0x0f0e);
		mii_mgr_write(priv, i, 24, 0x1610);
		mii_mgr_write(priv, i, 30, 0x1f15);
		mii_mgr_write(priv, i, 28, 0x6111);
	}

	/* 100Base AOI setting */
	mii_mgr_write(priv, 0, 31, 0x5000);	/* change G5 page */
	mii_mgr_write(priv, 0, 19, 0x004a);
	mii_mgr_write(priv, 0, 20, 0x015a);
	mii_mgr_write(priv, 0, 21, 0x00ee);
	mii_mgr_write(priv, 0, 22, 0x0033);
	mii_mgr_write(priv, 0, 23, 0x020a);
	mii_mgr_write(priv, 0, 24, 0x0000);
	mii_mgr_write(priv, 0, 25, 0x024a);
	mii_mgr_write(priv, 0, 26, 0x035a);
	mii_mgr_write(priv, 0, 27, 0x02ee);
	mii_mgr_write(priv, 0, 28, 0x0233);
	mii_mgr_write(priv, 0, 29, 0x000a);
	mii_mgr_write(priv, 0, 30, 0x0000);

	/* Fix EPHY idle state abnormal behavior */
	mii_mgr_write(priv, 0, 31, 0x4000);	/* change G4 page */
	mii_mgr_write(priv, 0, 29, 0x000d);
	mii_mgr_write(priv, 0, 30, 0x0500);
}

static void rt305x_esw_init(struct mt7628_esw_priv *priv)
{
	void __iomem *base = priv->base;

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	/*
	 * FC_RLS_TH=200, FC_SET_TH=160
	 * DROP_RLS=120, DROP_SET_TH=80
	 */
	writel(0xc8a07850, base + MT7628_SWITCH_FCT0);
	writel(0x00000000, base + MT7628_SWITCH_SGC2);
	writel(0x00405555, base + MT7628_SWITCH_PFC1);
	writel(0x00007f7f, base + MT7628_SWITCH_POC0);
	writel(0x00007f7f, base + MT7628_SWITCH_POC2);	/* disable VLAN */
	writel(0x0002500c, base + MT7628_SWITCH_FCT2);
	/* hashing algorithm=XOR48, aging interval=300sec */
	writel(0x0008a301, base + MT7628_SWITCH_SGC);
	writel(0x02404040, base + MT7628_SWITCH_SOCPC);

	/* Ext PHY Addr=0x1f */
	writel(0x3f502b28, base + MT7628_SWITCH_FPA1);
	writel(0x00000000, base + MT7628_SWITCH_FPA);
	/* 1us cycle number=125 (FE's clock=125Mhz) */
	writel(0x7d000000, base + MT7628_SWITCH_BMU_CTRL);

	/* Configure analog GPIO setup */
	rt_sysc_m32(MT7628_EPHY_P0_DIS, MT7628_EPHY_GPIO_AIO_EN,
		    MT7628_AGPIO_CFG_REG);

	/* Reset PHY */
	rt_sysc_m32(0, RSTCTRL_EPHY_RST, MT7628_RSTCTRL_REG);
	rt_sysc_m32(RSTCTRL_EPHY_RST, 0, MT7628_RSTCTRL_REG);
	mdelay(10);

	/* Set P0 EPHY LED mode */
	rt_sysc_m32(0x0ffc0ffc, 0x05540554, MT7628_GPIO2_MODE_REG);
	mdelay(10);

	mt7628_ephy_init(priv);
}

static int mt7628_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	struct mt7628_esw_priv *priv = ds->priv;
	u32 val;
	int ret;

	ret = mii_mgr_read(priv, port, regnum, &val);
	// test-only: ret error handling

	return val;
}

static int mt7628_phy_write(struct dsa_switch *ds, int port, int regnum,
			    u16 val)
{
	struct mt7628_esw_priv *priv = ds->priv;

	return mii_mgr_write(priv, port, regnum, val);
}

static const struct dsa_switch_ops mt7628_switch_ops = {
//	.get_tag_protocol	= mtk_get_tag_protocol,
	.setup			= mt7628_setup,
	.phy_read		= mt7628_phy_read,
	.phy_write		= mt7628_phy_write,
};

//static int mt7628_probe(struct mdio_device *mdiodev)
static int mt7628_probe(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
//	struct device_node *np = pdev->dev.of_node;
	struct mt7628_esw_priv *priv;
	int ret;

	printk("%s (%d)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", __func__, __LINE__); // test-only

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	printk("%s (%d): base=%pF\n", __func__, __LINE__, priv->base); // test-only
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	ret = device_reset(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "switch reset failed!\n");
		return ret;
	}

	priv->ds = dsa_switch_alloc(&pdev->dev, DSA_MAX_PORTS);
	if (!priv->ds)
		return -ENOMEM;

	// test-only: name and return code
	rt305x_esw_init(priv);

	priv->ds->priv = priv;
	priv->ds->dev = &pdev->dev;
	priv->ds->ops = &mt7628_switch_ops;
	dev_set_drvdata(&pdev->dev, priv->ds);

	return dsa_register_switch(priv->ds);
}

//static void mt7628_remove(struct mdio_device *mdiodev)
static int mt7628_remove(struct platform_device *pdev)
{
	struct mt7628_esw_priv *priv = dev_get_drvdata(&pdev->dev);

	printk("%s (%d)\n", __func__, __LINE__); // test-only
	dsa_unregister_switch(priv->ds);

	return 0;
}

static const struct of_device_id mt7628_of_match[] = {
	{ .compatible = "mediatek,mt7628-esw" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt7628_of_match);

//static struct mdio_driver mt7628_mdio_driver = {
static struct platform_driver mt7628_mdio_driver = {
	.probe  = mt7628_probe,
	.remove = mt7628_remove,
//	.mdiodrv.driver = {
	.driver = {
		.name = "mt7628-esw",
		.of_match_table = mt7628_of_match,
	},
};

//mdio_module_driver(mt7628_mdio_driver);
module_platform_driver(mt7628_mdio_driver);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("Driver for Mediatek MT7628 Switch (ESW)");
MODULE_LICENSE("GPL");
