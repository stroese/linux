// SPDX-License-Identifier: GPL-2.0-only
/*
 * Mediatek MT7628/88 ESW DSA Switch driver
 * Copyright (C) 2019 Stefan Roese <sr@denx.de>
 *
 * Based on the OpenWRT driver esw_rt3050.c, which is:
 *   Copyright (C) 2009-2015 John Crispin <blogic@openwrt.org>
 *   Copyright (C) 2009-2015 Felix Fietkau <nbd@nbd.name>
 *   Copyright (C) 2013-2015 Michael Lee <igvtee@gmail.com>
 *   Copyright (C) 2016 Vittorio Gambaletta <openwrt@vittgam.net>
 */
#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/netdevice.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <net/dsa.h>
#include <asm/mach-ralink/ralink_regs.h>

/* System controller register */
#define MT7628_RSTCTRL_REG		0x34
#define RSTCTRL_EPHY_RST		BIT(24)

#define MT7628_AGPIO_CFG_REG		0x3c
#define MT7628_EPHY_GPIO_AIO_EN		GENMASK(20, 17)
#define MT7628_EPHY_P0_DIS		BIT(16)

#define MT7628_GPIO2_MODE_REG		0x64

/* Ethernet switch register (ESW) */
#define MT7628_ESW_ISR			0x00
#define MT7628_ESW_IMR			0x04
#define MT7628_ESW_PORT_ST_CHG		BIT(26)

#define MT7628_ESW_FCT0			0x08

#define MT7628_ESW_PFC1			0x14
#define MT7628_ESW_PFC1_EN_VLAN_M	0xff
#define MT7628_ESW_PFC1_EN_VLAN_S	16
#define MT7628_ESW_PFC1_EN_TOS_S	24

#define MT7628_ESW_PVIDC(_n)		(0x40 + 4 * (_n))
#define MT7628_ESW_PVIDC_PVID_M		0xfff
#define MT7628_ESW_PVIDC_PVID_S		12

#define MT7628_ESW_VLANI(_n)		(0x50 + 4 * (_n))
#define MT7628_ESW_VLANI_VID_M		0xfff
#define MT7628_ESW_VLANI_VID_S		12

#define MT7628_ESW_VMSC(_n)		(0x70 + 4 * (_n))
#define MT7628_ESW_VMSC_MSC_M		0xff
#define MT7628_ESW_VMSC_MSC_S		8

#define MT7628_ESW_POA			0x80
#define MT7628_ESW_POA_LINK_MASK	0x1f
#define MT7628_ESW_POA_LINK_SHIFT	25

#define MT7628_ESW_FPA			0x84

#define MT7628_ESW_SOCPC		0x8c
#define MT7628_ESW_SOCPC_DISUN2CPU_S	0
#define MT7628_ESW_SOCPC_DISMC2CPU_S	8
#define MT7628_ESW_SOCPC_DISBC2CPU_S	16
#define MT7628_ESW_SOCPC_CRC_PADDING	BIT(25)

#define MT7628_ESW_POC0			0x90
#define MT7628_ESW_POC0_EN_BP_S		0
#define MT7628_ESW_POC0_EN_FC_S		8
#define MT7628_ESW_POC0_DIS_RMC2CPU_S	16
#define MT7628_ESW_POC0_DIS_PORT_M	0x7f
#define MT7628_ESW_POC0_DIS_PORT_S	23

#define MT7628_ESW_POC2			0x98
#define MT7628_ESW_POC2_UNTAG_EN_M	0xff
#define MT7628_ESW_POC2_UNTAG_EN_S	0
#define MT7628_ESW_POC2_ENAGING_S	8
#define MT7628_ESW_POC2_DIS_UC_PAUSE_S	16

#define MT7628_ESW_SGC			0x9c

#define MT7628_ESW_PCR0			0xc0
#define PCR0_PHY_ADDR			GENMASK(4, 0)
#define PCR0_PHY_REG			GENMASK(12, 8)
#define PCR0_WT_PHY_CMD			BIT(13)
#define PCR0_RD_PHY_CMD			BIT(14)
#define PCR0_WT_DATA			GENMASK(31, 16)

#define MT7628_ESW_PCR1			0xc4
#define PCR1_WT_DONE			BIT(0)
#define PCR1_RD_RDY			BIT(1)
#define PCR1_RD_DATA			GENMASK(31, 16)

// test-only: FPA1 vs FPA2 (documentation vs OpenWrt v4.14 driver)
#define MT7628_ESW_FPA2			0xc8

#define MT7628_ESW_FCT2			0xcc

#define MT7628_ESW_SGC2			0xe4
#define MT7628_ESW_SGC2_DOUBLE_TAG_M	0x7f
#define MT7628_ESW_SGC2_DOUBLE_TAG_S	0
#define MT7628_ESW_SGC2_LAN_PMAP_M	0x3f
#define MT7628_ESW_SGC2_LAN_PMAP_S	24

#define MT7628_ESW_P0LED		0xa4
#define MT7628_ESW_P1LED		0xa8
#define MT7628_ESW_P2LED		0xac
#define MT7628_ESW_P3LED		0xb0
#define MT7628_ESW_P4LED		0xb4

#define MT7628_ESW_VLAN_NONE		0xfff

#define MT7628_ESW_GSC_BC_STROM_MASK	0x3
#define MT7628_ESW_GSC_BC_STROM_SHIFT	4

#define MT7628_ESW_GSC_LED_FREQ_MASK	0x3
#define MT7628_ESW_GSC_LED_FREQ_SHIFT	23

#define MT7628_ESW_PORT0		0
#define MT7628_ESW_PORT1		1
#define MT7628_ESW_PORT2		2
#define MT7628_ESW_PORT3		3
#define MT7628_ESW_PORT4		4
#define MT7628_ESW_PORT5		5
#define MT7628_ESW_PORT6		6

#define MT7628_ESW_PORTS_NONE		0

#define MT7628_ESW_PMAP_LLLLLL		0x3f
#define MT7628_ESW_PMAP_LLLLWL		0x2f
#define MT7628_ESW_PMAP_WLLLLL		0x3e

#define MT7628_ESW_PORTS_INTERNAL				\
	(BIT(MT7628_ESW_PORT0) | BIT(MT7628_ESW_PORT1) |	\
	 BIT(MT7628_ESW_PORT2) | BIT(MT7628_ESW_PORT3) |	\
	 BIT(MT7628_ESW_PORT4))

#define MT7628_ESW_PORTS_NOCPU					\
	(MT7628_ESW_PORTS_INTERNAL | BIT(MT7628_ESW_PORT5))

#define MT7628_ESW_PORTS_CPU		BIT(MT7628_ESW_PORT6)

#define MT7628_ESW_PORTS_ALL				\
	(MT7628_ESW_PORTS_NOCPU | MT7628_ESW_PORTS_CPU)


#define MT7628_ESW_NUM_VLANS		16
#define MT7628_ESW_NUM_VIDS		4096
#define MT7628_ESW_NUM_PORTS		7
#define MT7628_ESW_NUM_LANWAN		6
#define MT7628_ESW_NUM_LEDS		5

#define MT7628_ESW_CPU_PORT		6

#define CONFIG_MDIO_TIMEOUT		100

struct mt7628_esw_port {
	bool	enable;
	bool	doubletag;
	bool	untag;
	u8	led;
	u16	pvid;
};

struct mt7628_esw_vlan {
	u8	ports;
	u16	vid;
};

struct mt7628_esw_priv {
	void __iomem *base;		/* switch base address */

	struct dsa_switch *ds;

	struct mii_dev *bus;

	// test-only: all needed?
	bool global_vlan_enable;
	bool alt_vlan_disable;
	int bc_storm_protect;
	int led_frequency;

	struct mt7628_esw_vlan vlans[MT7628_ESW_NUM_VLANS];
	struct mt7628_esw_port ports[MT7628_ESW_NUM_PORTS];

	/* Protects against concurrent register r/w operations. */
	spinlock_t reg_rw_lock;
};

static int mdio_wait_read(struct mt7628_esw_priv *priv, u32 mask, bool mask_set)
{
	void __iomem *base = priv->base;
	int ret = 0;
	u32 val;

	ret = readl_poll_timeout(base + MT7628_ESW_PCR1, val,
				 mask_set ? (val & mask) : !(val & mask),
				 1, CONFIG_MDIO_TIMEOUT * 1000);
	if (ret)
		dev_err(priv->ds->dev, "timed out waiting for idle\n");

	return ret;
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
	       base + MT7628_ESW_PCR0);

	/* Make sure previous read operation is complete */
	ret = mdio_wait_read(priv, PCR1_RD_RDY, true);
	if (ret)
		return ret;

	status = readl(base + MT7628_ESW_PCR1);
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
	writel(data, base + MT7628_ESW_PCR0);

	return mdio_wait_read(priv, PCR1_WT_DONE, true);
}

static void mt7628_esw_rmw_raw(struct mt7628_esw_priv *priv, unsigned reg,
			       unsigned long mask, unsigned long val)
{
	unsigned long t;

	t = __raw_readl(priv->base + reg) & ~mask;
	__raw_writel(t | val, priv->base + reg);
}

static void mt7628_esw_rmw(struct mt7628_esw_priv *priv, unsigned reg,
			   unsigned long mask, unsigned long val)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->reg_rw_lock, flags);
	mt7628_esw_rmw_raw(priv, reg, mask, val);
	spin_unlock_irqrestore(&priv->reg_rw_lock, flags);
}

static void mt7628_esw_set_vlan_id(struct mt7628_esw_priv *priv,
				   unsigned vlan, unsigned vid)
{
	unsigned s;

	s = MT7628_ESW_VLANI_VID_S * (vlan % 2);
	mt7628_esw_rmw(priv,
		       MT7628_ESW_VLANI(vlan / 2), MT7628_ESW_VLANI_VID_M << s,
		       (vid & MT7628_ESW_VLANI_VID_M) << s);
}

static void mt7628_esw_set_vmsc(struct mt7628_esw_priv *priv,
				unsigned vlan, unsigned msc)
{
	unsigned s;

	s = MT7628_ESW_VMSC_MSC_S * (vlan % 4);
	mt7628_esw_rmw(priv,
		       MT7628_ESW_VMSC(vlan / 4), MT7628_ESW_VMSC_MSC_M << s,
		       (msc & MT7628_ESW_VMSC_MSC_M) << s);
}

static void mt7628_esw_set_pvid(struct mt7628_esw_priv *priv,
				unsigned port, unsigned pvid)
{
	unsigned s;

	s = MT7628_ESW_PVIDC_PVID_S * (port % 2);
	mt7628_esw_rmw(priv,
		       MT7628_ESW_PVIDC(port / 2), MT7628_ESW_PVIDC_PVID_M << s,
		       (pvid & MT7628_ESW_PVIDC_PVID_M) << s);
}

static void mt7628_esw_set_gsc(struct mt7628_esw_priv *priv)
{
	mt7628_esw_rmw(priv, MT7628_ESW_SGC,
		       MT7628_ESW_GSC_BC_STROM_MASK <<
		       MT7628_ESW_GSC_BC_STROM_SHIFT,
		       priv->bc_storm_protect << MT7628_ESW_GSC_BC_STROM_SHIFT);
	mt7628_esw_rmw(priv, MT7628_ESW_SGC,
		       MT7628_ESW_GSC_LED_FREQ_MASK <<
		       MT7628_ESW_GSC_LED_FREQ_SHIFT,
		       priv->led_frequency << MT7628_ESW_GSC_LED_FREQ_SHIFT);
}

static unsigned mt7628_esw_get_port_disable(struct mt7628_esw_priv *priv)
{
	unsigned reg;

	reg = readl(priv->base + MT7628_ESW_POC0);
	return (reg >> MT7628_ESW_POC0_DIS_PORT_S) &
		MT7628_ESW_POC0_DIS_PORT_M;
}

static void mt7628_esw_set_port_disable(struct mt7628_esw_priv *priv,
					unsigned disable_mask)
{
	unsigned old_mask;
	unsigned enable_mask;
	unsigned changed;
	int i;

//	printk("%s (%d): disable_mask=%08x\n", __func__, __LINE__, disable_mask); // test-only
	old_mask = mt7628_esw_get_port_disable(priv);
	changed = old_mask ^ disable_mask;
	enable_mask = old_mask & disable_mask;

	/* enable before writing to MII */
	mt7628_esw_rmw(priv, MT7628_ESW_POC0,
		       MT7628_ESW_POC0_DIS_PORT_M << MT7628_ESW_POC0_DIS_PORT_S,
		       enable_mask << MT7628_ESW_POC0_DIS_PORT_S);

	for (i = 0; i < MT7628_ESW_NUM_LEDS; i++) {
		if (!(changed & (1 << i)))
			continue;
		if (disable_mask & (1 << i)) {
			/* disable */
			mii_mgr_write(priv, i, MII_BMCR, BMCR_PDOWN);
		} else {
			/* enable */
			mii_mgr_write(priv, i, MII_BMCR,
				      BMCR_FULLDPLX | BMCR_ANENABLE |
				      BMCR_ANRESTART | BMCR_SPEED100);
		}
	}

	/* disable after writing to MII */
	mt7628_esw_rmw(priv, MT7628_ESW_POC0,
		       MT7628_ESW_POC0_DIS_PORT_M << MT7628_ESW_POC0_DIS_PORT_S,
		       disable_mask << MT7628_ESW_POC0_DIS_PORT_S);
}

static int mt7628_esw_apply_config(struct mt7628_esw_priv *priv)
{
	u8 disable = 0;
	u8 doubletag = 0;
	u8 en_vlan = 0;
	u8 untag = 0;
	int i;

	for (i = 0; i < MT7628_ESW_NUM_VLANS; i++) {
		u32 vid, vmsc;

		if (priv->global_vlan_enable) {
			vid = priv->vlans[i].vid;
			vmsc = priv->vlans[i].ports;
		} else {
			vid = MT7628_ESW_VLAN_NONE;
			vmsc = MT7628_ESW_PORTS_NONE;
		}

		mt7628_esw_set_vlan_id(priv, i, vid);
		mt7628_esw_set_vmsc(priv, i, vmsc);
	}

	for (i = 0; i < MT7628_ESW_NUM_PORTS; i++) {
		u32 pvid;

		disable |= !priv->ports[i].enable << i;

		if (priv->global_vlan_enable) {
			doubletag |= priv->ports[i].doubletag << i;
			en_vlan   |= 1                        << i;
			untag     |= priv->ports[i].untag     << i;
			pvid       = priv->ports[i].pvid;
		} else {
			int x = priv->alt_vlan_disable ? 0 : 1;
			doubletag |= x << i;
			en_vlan   |= x << i;
			untag     |= x << i;
			pvid       = 0;
		}

		mt7628_esw_set_pvid(priv, i, pvid);

		if (i < MT7628_ESW_NUM_LEDS) {
			writel(priv->ports[i].led,
			       priv->base + MT7628_ESW_P0LED + 4 * i);
		}
	}

	mt7628_esw_set_gsc(priv);
	mt7628_esw_set_port_disable(priv, disable);
	mt7628_esw_rmw(priv, MT7628_ESW_SGC2,
		       (MT7628_ESW_SGC2_DOUBLE_TAG_M << MT7628_ESW_SGC2_DOUBLE_TAG_S),
		       doubletag << MT7628_ESW_SGC2_DOUBLE_TAG_S);
	mt7628_esw_rmw(priv, MT7628_ESW_PFC1,
		       MT7628_ESW_PFC1_EN_VLAN_M << MT7628_ESW_PFC1_EN_VLAN_S,
		       en_vlan << MT7628_ESW_PFC1_EN_VLAN_S);
	mt7628_esw_rmw(priv, MT7628_ESW_POC2,
		       MT7628_ESW_POC2_UNTAG_EN_M << MT7628_ESW_POC2_UNTAG_EN_S,
		       untag << MT7628_ESW_POC2_UNTAG_EN_S);

	if (!priv->global_vlan_enable) {
		/*
		 * Still need to put all ports into vlan 0 or they'll be
		 * isolated.
		 * NOTE: vlan 0 is special, no vlan tag is prepended
		 */
		mt7628_esw_set_vlan_id(priv, 0, 0);
		mt7628_esw_set_vmsc(priv, 0, MT7628_ESW_PORTS_ALL);
	}

	return 0;
}

static int mt7628_esw_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	struct mt7628_esw_priv *priv = ds->priv;
	u32 val;
	int ret;

	ret = mii_mgr_read(priv, port, regnum, &val);
	if (ret == 0xffff)
		return 0xffff;

	return val;
}

static int mt7628_esw_phy_write(struct dsa_switch *ds, int port, int regnum,
			    u16 val)
{
	struct mt7628_esw_priv *priv = ds->priv;

	return mii_mgr_write(priv, port, regnum, val);
}

static enum dsa_tag_protocol mt7628_esw_get_tag_protocol(struct dsa_switch *ds,
							 int port)
{
	if (port != MT7628_ESW_CPU_PORT) {
		dev_warn(ds->dev, "port not matched with tagging CPU port\n");
		return DSA_TAG_PROTO_NONE;
	} else {
		// test-only: don't use custom tag for now
		return DSA_TAG_PROTO_NONE;
	}
}

static int mt7628_esw_setup(struct dsa_switch *ds)
{
	struct mt7628_esw_priv *priv = ds->priv;
	void __iomem *base = priv->base;
	int ret;
	int i;

	ret = device_reset(priv->ds->dev);
	if (ret) {
		dev_err(priv->ds->dev, "switch reset failed!\n");
		return ret;
	}
	mdelay(10);

	/* Vodoo from original driver */
	writel(0xc8a07850, base + MT7628_ESW_FCT0);
	writel(0x00000000, base + MT7628_ESW_SGC2);

	/* Port priority 1 for all ports, vlan enabled */
	writel(0x00005555 | (MT7628_ESW_PORTS_ALL << MT7628_ESW_PFC1_EN_VLAN_S),
	       base + MT7628_ESW_PFC1);

	/* Enable all ports, Back Pressure and Flow Control */
	writel(((MT7628_ESW_PORTS_ALL << MT7628_ESW_POC0_EN_BP_S) |
		(MT7628_ESW_PORTS_ALL << MT7628_ESW_POC0_EN_FC_S)),
	       base + MT7628_ESW_POC0);

	/* Enable Aging, and VLAN TAG removal */
	writel(((MT7628_ESW_PORTS_ALL << MT7628_ESW_POC2_ENAGING_S) |
		(MT7628_ESW_PORTS_NOCPU << MT7628_ESW_POC2_UNTAG_EN_S)),
	       base + MT7628_ESW_POC2);

	writel(0x0002500c, base + MT7628_ESW_FCT2);

	/* 300s aging timer, max packet len 1536, broadcast storm prevention
	 * disabled, disable collision abort, mac xor48 hash, 10 packet back
	 * pressure jam, GMII disable was_transmit, back pressure disabled,
	 * 30ms led flash, unmatched IGMP as broadcast, rmc tb fault to all
	 * ports.
	 */
	writel(0x0008a301, base + MT7628_ESW_SGC);

	/* Setup SoC Port control register */
	writel((MT7628_ESW_SOCPC_CRC_PADDING |
		(MT7628_ESW_PORTS_CPU << MT7628_ESW_SOCPC_DISUN2CPU_S) |
		(MT7628_ESW_PORTS_CPU << MT7628_ESW_SOCPC_DISMC2CPU_S) |
		(MT7628_ESW_PORTS_CPU << MT7628_ESW_SOCPC_DISBC2CPU_S)),
	       base + MT7628_ESW_SOCPC);

	/* Ext phy base addr 31, enable port 5 polling, rx/tx clock skew 1,
	 * turbo mii off, rgmi 3.3v off
	 * port5: disabled
	 * port6: enabled, gige, full-duplex, rx/tx-flow-control
	 */
	writel(0x3f502b28, base + MT7628_ESW_FPA2);
	writel(0x00000000, base + MT7628_ESW_FPA);

	/* Force Link/Activity on ports */
	writel(0x00000005, base + MT7628_ESW_P0LED);
	writel(0x00000005, base + MT7628_ESW_P1LED);
	writel(0x00000005, base + MT7628_ESW_P2LED);
	writel(0x00000005, base + MT7628_ESW_P3LED);
	writel(0x00000005, base + MT7628_ESW_P4LED);

	priv->ports[0].enable = true;
	priv->ports[MT7628_ESW_CPU_PORT].enable = true;

	/* Reset EPHY */
	rt_sysc_m32(0, RSTCTRL_EPHY_RST, MT7628_RSTCTRL_REG);
	rt_sysc_m32(RSTCTRL_EPHY_RST, 0, MT7628_RSTCTRL_REG);
	mdelay(10);

	mii_mgr_write(priv, 0, 31, 0x2000); /* change G2 page */
	mii_mgr_write(priv, 0, 26, 0x0020);

	// test-only: only MT7628/88 stuff here right now imported from
	// OpenWrt v4.14 ESW driver
	for (i = 0; i < 5; i++) {
		mii_mgr_write(priv, i, 31, 0x8000);
		mii_mgr_write(priv, i,  0, 0x3100);
		mii_mgr_write(priv, i, 30, 0xa000);
		mii_mgr_write(priv, i, 31, 0xa000);
		mii_mgr_write(priv, i, 16, 0x0606);
		mii_mgr_write(priv, i, 23, 0x0f0e);
		mii_mgr_write(priv, i, 24, 0x1610);
		mii_mgr_write(priv, i, 30, 0x1f15);
		mii_mgr_write(priv, i, 28, 0x6111);
		mii_mgr_write(priv, i, 31, 0x2000);
		mii_mgr_write(priv, i, 26, 0x0000);
	}

	/* 100Base AOI setting */
	mii_mgr_write(priv, 0, 31, 0x5000);
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

	/* Unused HW feature, but still nice to be consistent here...
	 * This is also exported to userspace ('lan' attribute) so it's
	 * conveniently usable to decide which ports go into the wan vlan by
	 * default.
	 */
	mt7628_esw_rmw(priv, MT7628_ESW_SGC2,
		       MT7628_ESW_SGC2_LAN_PMAP_M << MT7628_ESW_SGC2_LAN_PMAP_S,
		       MT7628_ESW_PMAP_LLLLLL << MT7628_ESW_SGC2_LAN_PMAP_S);

	/* Make the switch leds blink */
	for (i = 0; i < MT7628_ESW_NUM_LEDS; i++)
		priv->ports[i].led = 0x05;

	/* Apply the empty config */
	mt7628_esw_apply_config(priv);

	/* Only unmask the port change interrupt */
	writel(MT7628_ESW_PORT_ST_CHG, priv->base + MT7628_ESW_ISR);
	writel(~MT7628_ESW_PORT_ST_CHG, base + MT7628_ESW_IMR);

	return 0;
}

static irqreturn_t mt7628_esw_interrupt(int irq, void *_priv)
{
	struct mt7628_esw_priv *priv = (struct mt7628_esw_priv *)_priv;
	u32 status;
	int i;

	status = readl(priv->base + MT7628_ESW_ISR);
	if (status & MT7628_ESW_PORT_ST_CHG) {
		u32 link = readl(priv->base + MT7628_ESW_POA);
//		printk("%s (%d): link=%08x\n", __func__, __LINE__, link); // test-only

		link >>= MT7628_ESW_POA_LINK_SHIFT;
		link &= MT7628_ESW_POA_LINK_MASK;

		for (i = 0; i < MT7628_ESW_NUM_LEDS; i++) {
			const struct dsa_port *dp = dsa_to_port(priv->ds, i);
			const struct dsa_port *dp_cpu =
				dsa_to_port(priv->ds, MT7628_ESW_CPU_PORT);

			/* Skip disabled ports */
			if (!priv->ports[i].enable)
				continue;

			dev_info(priv->ds->dev, "port %d link changed 0x%x\n",
				 i, link);

			if (link & BIT(i))
				netif_carrier_on(dp->slave);
			else
				netif_carrier_off(dp->slave);

			/*
			 * On the IOT mode (only PHY, no switch used) also
			 * set link on the master net node (eth0)
			 */
			if (link & BIT(i))
				netif_carrier_on(dp_cpu->slave);
			else
				netif_carrier_off(dp_cpu->slave);
		}
	}

	writel(status, priv->base + MT7628_ESW_ISR);

	return IRQ_HANDLED;
}

//static int mt7628_esw_port_enable(struct mt7628_esw_priv *priv, int port)
static int mt7628_esw_port_enable(struct dsa_switch *ds, int port,
				  struct phy_device *phy)
{
	struct mt7628_esw_priv *priv = ds->priv;

//	printk("%s (%d): port=%d\n", __func__, __LINE__, port); // test-only
	/* Skip port 0 enabling */
	if (port == 0)
		return -ENODEV;

	priv->ports[port].enable = true;

	/* Apply the new configuration */
	mt7628_esw_apply_config(priv);

	return 0;
}

//static int mt7628_esw_port_disable(struct mt7628_esw_priv *priv, int port)
static void mt7628_esw_port_disable(struct dsa_switch *ds, int port)
{
	struct mt7628_esw_priv *priv = ds->priv;

//	printk("%s (%d): port=%d\n", __func__, __LINE__, port); // test-only
	priv->ports[port].enable = false;

	/* Apply the new configuration */
	mt7628_esw_apply_config(priv);
}

static int mt7628_esw_port_bridge_join(struct dsa_switch *ds, int port,
				       struct net_device *bridge)
{
//	printk("%s (%d)\n", __func__, __LINE__); // test-only
	return 0;
}

static void mt7628_esw_port_bridge_leave(struct dsa_switch *ds, int port,
					 struct net_device *bridge)
{
//	printk("%s (%d)\n", __func__, __LINE__); // test-only
}

static const struct dsa_switch_ops mt7628_esw_ops = {
	.get_tag_protocol	= mt7628_esw_get_tag_protocol,
	.setup			= mt7628_esw_setup,
	.phy_read		= mt7628_esw_phy_read,
	.phy_write		= mt7628_esw_phy_write,

	.port_enable		= mt7628_esw_port_enable,
	.port_disable		= mt7628_esw_port_disable,

	.port_bridge_join	= mt7628_esw_port_bridge_join,
	.port_bridge_leave	= mt7628_esw_port_bridge_leave,
};

static int mt7628_esw_probe(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct device_node *dn = pdev->dev.of_node;
	struct mt7628_esw_priv *priv;
	int irq;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->ds = devm_kzalloc(&pdev->dev, sizeof(*priv->ds), GFP_KERNEL);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->dev = &pdev->dev;
	priv->ds->num_ports = MT7628_ESW_NUM_PORTS;

	/* Clean and disable port status change interrupt */
	writel(MT7628_ESW_PORT_ST_CHG, priv->base + MT7628_ESW_ISR);
	writel(~MT7628_ESW_PORT_ST_CHG, priv->base + MT7628_ESW_IMR);

	irq = irq_of_parse_and_map(dn, 0);

	ret = devm_request_irq(&pdev->dev, irq, mt7628_esw_interrupt, 0,
			       "esw", priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request ESW IRQ\n");
		return ret;
	}

	priv->ds->priv = priv;
	priv->ds->dev = &pdev->dev;
	priv->ds->ops = &mt7628_esw_ops;
	dev_set_drvdata(&pdev->dev, priv->ds);

	spin_lock_init(&priv->reg_rw_lock);

	return dsa_register_switch(priv->ds);
}

static int mt7628_esw_remove(struct platform_device *pdev)
{
	struct mt7628_esw_priv *priv = dev_get_drvdata(&pdev->dev);

	dsa_unregister_switch(priv->ds);

	return 0;
}

static const struct of_device_id mt7628_esw_of_match[] = {
	{ .compatible = "ralink,rt3050-esw" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt7628_of_match);

static struct platform_driver mt7628_mdio_driver = {
	.probe  = mt7628_esw_probe,
	.remove = mt7628_esw_remove,
	.driver = {
		.name = "mt7628-esw",
		.of_match_table = mt7628_esw_of_match,
	},
};

module_platform_driver(mt7628_mdio_driver);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("Driver for Mediatek MT7628 Switch (ESW)");
MODULE_LICENSE("GPL");
