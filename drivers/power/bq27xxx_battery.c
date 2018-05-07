/*
 * BQ27xxx battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Datasheets:
 * http://www.ti.com/product/bq27000
 * http://www.ti.com/product/bq27200
 * http://www.ti.com/product/bq27010
 * http://www.ti.com/product/bq27210
 * http://www.ti.com/product/bq27500
 * http://www.ti.com/product/bq27510-g3
 * http://www.ti.com/product/bq27520-g4
 * http://www.ti.com/product/bq27530-g1
 * http://www.ti.com/product/bq27531-g1
 * http://www.ti.com/product/bq27541-g1
 * http://www.ti.com/product/bq27542-g1
 * http://www.ti.com/product/bq27546-g1
 * http://www.ti.com/product/bq27742-g1
 * http://www.ti.com/product/bq27545-g1
 * http://www.ti.com/product/bq27421-g1
 * http://www.ti.com/product/bq27425-g1
 * http://www.ti.com/product/bq27411-g1
 * http://www.ti.com/product/bq27621-g1
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>

#include <linux/power/bq27xxx_battery.h>
#include <mach/battery_common.h>

#define DRIVER_VERSION		"1.2.0"

#define BQ27XXX_MANUFACTURER	"Texas Instruments"

/* BQ27XXX Flags */
#define BQ27XXX_FLAG_DSC	BIT(0)
#define BQ27XXX_FLAG_SOCF	BIT(1) /* State-of-Charge threshold final */
#define BQ27XXX_FLAG_SOC1	BIT(2) /* State-of-Charge threshold 1 */
#define BQ27XXX_FLAG_FC		BIT(9)
#define BQ27XXX_FLAG_OTD	BIT(14)
#define BQ27XXX_FLAG_OTC	BIT(15)
#define BQ27XXX_FLAG_UT		BIT(14)
#define BQ27XXX_FLAG_OT		BIT(15)

/* BQ27000 has different layout for Flags register */
#define BQ27000_FLAG_EDVF	BIT(0) /* Final End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_EDV1	BIT(1) /* First End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_CI		BIT(4) /* Capacity Inaccurate flag */
#define BQ27000_FLAG_FC		BIT(5)
#define BQ27000_FLAG_CHGS	BIT(7) /* Charge state flag */

#define BQ27XXX_RS			(20) /* Resistor sense mOhm */
#define BQ27XXX_POWER_CONSTANT		(29200) /* 29.2 µV^2 * 1000 */
#define BQ27XXX_CURRENT_CONSTANT	(3570) /* 3.57 µV * 1000 */

struct bq27xxx_device_info;
struct bq27xxx_access_methods {
	int (*read)(struct bq27xxx_device_info *di, u8 reg, bool single);
	int (*write)(struct bq27xxx_device_info *di, u8 reg, int value,
			bool single);
	int (*blk_read)(struct bq27xxx_device_info *di, u8 reg, u8 *data,
		u8 sz);
	int (*blk_write)(struct bq27xxx_device_info *di, u8 reg, u8 *data,
		u8 sz);
};

#define INVALID_REG_ADDR	0xff

/*
 * bq27xxx_reg_index - Register names
 *
 * These are indexes into a device's register mapping array.
 */
enum bq27xxx_reg_index {
	BQ27XXX_REG_CTRL = 0,	/* Control */
	BQ27XXX_REG_TEMP,	/* Temperature */
	BQ27XXX_REG_INT_TEMP,	/* Internal Temperature */
	BQ27XXX_REG_VOLT,	/* Voltage */
	BQ27XXX_REG_AI,		/* Average Current */
	BQ27XXX_REG_FLAGS,	/* Flags */
	BQ27XXX_REG_TTE,	/* Time-to-Empty */
	BQ27XXX_REG_TTF,	/* Time-to-Full */
	BQ27XXX_REG_TTES,	/* Time-to-Empty Standby */
	BQ27XXX_REG_TTECP,	/* Time-to-Empty at Constant Power */
	BQ27XXX_REG_NAC,	/* Nominal Available Capacity */
	BQ27XXX_REG_FCC,	/* Full Charge Capacity */
	BQ27XXX_REG_CYCT,	/* Cycle Count */
	BQ27XXX_REG_AE,		/* Available Energy */
	BQ27XXX_REG_SOC,	/* State-of-Charge */
	BQ27XXX_REG_DCAP,	/* Design Capacity */
	BQ27XXX_REG_AP,		/* Average Power */
	BQ27XXX_REG_FAC,
	BQ27XXX_REG_RMC,
	BQ27XXX_REG_AVG_CURR,
	BQ27XXX_REG_RCU,
	BQ27XXX_REG_FCCU,
	BQ27XXX_REG_FCCF,
	BQ27XXX_REG_SOCU,
	NUM_REGS
};

struct bq27xxx_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int energy;
	int flags;
	int power_avg;
	int health;
};

struct dm_reg {
	u8 subclass;
	u8 offset;
	u8 len;
	u32 data;
};

struct bq27xxx_device_info {
	struct device		*dev;
	int			id;
	enum bq27xxx_chip	chip;

	struct bq27xxx_reg_cache cache;
	int charge_design_full;

	unsigned long last_update;
	struct delayed_work work;

	struct power_supply	*bat;

	struct bq27xxx_access_methods bus;

	struct mutex lock;

	u8 *regs;
	struct dm_reg *dm_regs;
	u16 dm_regs_count;
	int fake_capacity;
};

/* Register mappings */
static u8 bq27000_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	INVALID_REG_ADDR,	/* INT TEMP - NA*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0a,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0x18,	/* TTF		*/
	0x1c,	/* TTES		*/
	0x26,	/* TTECP	*/
	0x0c,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2a,	/* CYCT		*/
	0x22,	/* AE		*/
	0x0b,	/* SOC(RSOC)	*/
	0x76,	/* DCAP(ILMD)	*/
	0x24,	/* AP		*/
};

static u8 bq27010_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	INVALID_REG_ADDR,	/* INT TEMP - NA*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0a,	/* FLAGS	*/
	0x16,	/* TTE		*/
	0x18,	/* TTF		*/
	0x1c,	/* TTES		*/
	0x26,	/* TTECP	*/
	0x0c,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2a,	/* CYCT		*/
	INVALID_REG_ADDR,	/* AE - NA	*/
	0x0b,	/* SOC(RSOC)	*/
	0x76,	/* DCAP(ILMD)	*/
	INVALID_REG_ADDR,	/* AP - NA	*/
};

static u8 bq27500_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0x28,	/* INT TEMP	*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0a,	/* FLAGS	*/
	0x16,	/* TTE		*/
	INVALID_REG_ADDR,	/* TTF - NA	*/
	0x1a,	/* TTES		*/
	INVALID_REG_ADDR,	/* TTECP - NA	*/
	0x0c,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x1e,	/* CYCT		*/
	INVALID_REG_ADDR,	/* AE - NA	*/
	0x20,	/* SOC(RSOC)	*/
	0x2e,	/* DCAP(ILMD)	*/
	INVALID_REG_ADDR,	/* AP - NA	*/
};

static u8 bq27530_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0x32,	/* INT TEMP	*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0a,	/* FLAGS	*/
	0x16,	/* TTE		*/
	INVALID_REG_ADDR,	/* TTF - NA	*/
	INVALID_REG_ADDR,	/* TTES - NA	*/
	INVALID_REG_ADDR,	/* TTECP - NA	*/
	0x0c,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2a,	/* CYCT		*/
	INVALID_REG_ADDR,	/* AE - NA	*/
	0x2c,	/* SOC(RSOC)	*/
	INVALID_REG_ADDR,	/* DCAP - NA	*/
	0x24,	/* AP		*/
};

static u8 bq27541_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0x28,	/* INT TEMP	*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0a,	/* FLAGS	*/
	0x16,	/* TTE		*/
	INVALID_REG_ADDR,	/* TTF - NA	*/
	INVALID_REG_ADDR,	/* TTES - NA	*/
	INVALID_REG_ADDR,	/* TTECP - NA	*/
	0x0c,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2a,	/* CYCT		*/
	INVALID_REG_ADDR,	/* AE - NA	*/
	0x2c,	/* SOC(RSOC)	*/
	0x3c,	/* DCAP		*/
	0x76,	/* AP		*/
};

static u8 bq27545_regs[] = {
	0x00,	/* CONTROL	*/
	0x06,	/* TEMP		*/
	0x28,	/* INT TEMP	*/
	0x08,	/* VOLT		*/
	0x14,	/* AVG CURR	*/
	0x0a,	/* FLAGS	*/
	0x16,	/* TTE		*/
	INVALID_REG_ADDR,	/* TTF - NA	*/
	INVALID_REG_ADDR,	/* TTES - NA	*/
	INVALID_REG_ADDR,	/* TTECP - NA	*/
	0x0c,	/* NAC		*/
	0x12,	/* LMD(FCC)	*/
	0x2a,	/* CYCT		*/
	INVALID_REG_ADDR,	/* AE - NA	*/
	0x2c,	/* SOC(RSOC)	*/
	INVALID_REG_ADDR,	/* DCAP - NA */
	0x24,	/* AP		*/
};

static u8 bq27421_regs[] = {
	0x00,	/* CONTROL	*/
	0x02,	/* TEMP		*/
	0x1e,	/* INT TEMP	*/
	0x04,	/* VOLT		*/
	0x10,	/* AVG CURR	*/
	0x06,	/* FLAGS	*/
	INVALID_REG_ADDR,	/* TTE - NA	*/
	INVALID_REG_ADDR,	/* TTF - NA	*/
	INVALID_REG_ADDR,	/* TTES - NA	*/
	INVALID_REG_ADDR,	/* TTECP - NA	*/
	0x08,	/* NAC		*/
	0x0e,	/* FCC		*/
	INVALID_REG_ADDR,	/* CYCT - NA	*/
	INVALID_REG_ADDR,	/* AE - NA	*/
	0x1c,	/* SOC		*/
	0x3c,	/* DCAP		*/
	0x18,	/* AP		*/
	0x0A,	/* FAC		*/
	0x0C,	/* RMC		*/
	0x10,	/* AVG CURR	*/
	0x28,	/* RCU		*/
	0x2C,	/* FCCU		*/
	0x2E,   /* FCCF*/
	0x30,	/* SOCU		*/
};

static u8 *bq27xxx_regs[] = {
	[BQ27000] = bq27000_regs,
	[BQ27010] = bq27010_regs,
	[BQ27500] = bq27500_regs,
	[BQ27530] = bq27530_regs,
	[BQ27541] = bq27541_regs,
	[BQ27545] = bq27545_regs,
	[BQ27421] = bq27421_regs,
};

static enum power_supply_property bq27000_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27010_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27500_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27530_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27541_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27545_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27421_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

/* bq274xx/bq276xx specific command information */
#define BQ274XX_UNSEAL_KEY		0x80008000
#define BQ274XX_RESET			0x41
#define BQ274XX_SOFT_RESET		0x42
#define BQ274XX_EXIT_CFGUPDATE		0x43

/*
 * SBS Commands for DF access - these are pretty standard
 * So, no need to go in the command array
 */
#define BLOCK_DATA_CLASS		0x3E
#define DATA_BLOCK			0x3F
#define BLOCK_DATA			0x40
#define BLOCK_DATA_CHECKSUM		0x60
#define BLOCK_DATA_CONTROL		0x61

/* Subcommands of Control() */
#define CONTROL_STATUS_SUBCMD		0x0000

#define BQ274XX_FLAG_ITPOR				0x20
#define BQ274XX_CTRL_STATUS_INITCOMP	0x80
#define SET_CFGUPDATE_SUBCMD		0x0013
#define SEAL_SUBCMD			0x0020


#define SPM_TIMEOUT  10*60 //10minutes

/*
 * Ordering the parameters based on subclass and then offset will help in
 * having fewer flash writes while updating.
 * Customize these values and, if necessary, add more based on system needs.
 */
static struct dm_reg bq274xx_dm_regs[] = {
	{64, 2, 1, 0x0F},	/* Op Config B */
	{80, 78, 1, 10},	/* TermV Valid t 10s */
	{82, 0, 2, 17312},	/* Qmax */
	{82, 3, 2, 15},		/* Reserve capacity */
	{82, 10, 2, 300},	/* Design Capacity */
	{82, 12, 2, 1140},	/* Design Energy */
	{82, 16, 2, 3300},	/* Terminate Voltage */
	{82, 27, 2, 33},	/* Taper rate */
};

#define BQ27XXX_PROP(_id, _prop)		\
	[_id] = {				\
		.props = _prop,			\
		.size = ARRAY_SIZE(_prop),	\
	}

static struct {
	enum power_supply_property *props;
	size_t size;
} bq27xxx_battery_props[] = {
	BQ27XXX_PROP(BQ27000, bq27000_battery_props),
	BQ27XXX_PROP(BQ27010, bq27010_battery_props),
	BQ27XXX_PROP(BQ27500, bq27500_battery_props),
	BQ27XXX_PROP(BQ27530, bq27530_battery_props),
	BQ27XXX_PROP(BQ27541, bq27541_battery_props),
	BQ27XXX_PROP(BQ27545, bq27545_battery_props),
	BQ27XXX_PROP(BQ27421, bq27421_battery_props),
};

extern PMU_ChargerStruct BMT_status;
static int mCapacity = 0;
static unsigned int poll_interval = 360;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval,
		 "battery poll interval in seconds - 0 disables polling");

/*
 * Common code for BQ27xxx devices
 */

static inline int bq27xxx_read(struct bq27xxx_device_info *di, int reg_index,
			       bool single)
{
	/* Reports EINVAL for invalid/missing registers */
	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return -EINVAL;

	return di->bus.read(di, di->regs[reg_index], single);
}

/*
 * Return the battery State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_soc(struct bq27xxx_device_info *di)
{
	int soc;

	soc = bq27xxx_read(di, BQ27XXX_REG_SOC, false);

	if (soc < 0)
		dev_dbg(di->dev, "error reading State-of-Charge\n");

	return soc;
}

/*
 * Return the battery State-of-Charge Unfiltered
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_socu(struct bq27xxx_device_info *di)
{
	int socu;

	socu = bq27xxx_read(di, BQ27XXX_REG_SOCU, false);

	if (socu < 0)
		dev_dbg(di->dev, "error reading State-of-Charge\n");

	return socu;
}


/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_charge(struct bq27xxx_device_info *di, u8 reg)
{
	int charge;

	charge = bq27xxx_read(di, reg, false);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	if (di->chip == BQ27000 || di->chip == BQ27010)
		charge *= BQ27XXX_CURRENT_CONSTANT / BQ27XXX_RS;
	else
		charge *= 1000;

	return charge;
}

/*
 * Return the battery Nominal available capacity in µAh
 * Or < 0 if something fails.
 */
static inline int bq27xxx_battery_read_nac(struct bq27xxx_device_info *di)
{
	int flags;

	if (di->chip == BQ27000 || di->chip == BQ27010) {
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, true);
		if (flags >= 0 && (flags & BQ27000_FLAG_CI))
			return -ENODATA;
	}

	return bq27xxx_battery_read_charge(di, BQ27XXX_REG_NAC);
}

/*
 * Return the battery Full Charge Capacity in µAh
 * Or < 0 if something fails.
 */
static inline int bq27xxx_battery_read_fcc(struct bq27xxx_device_info *di)
{
	return bq27xxx_battery_read_charge(di, BQ27XXX_REG_FCC);
}

/*
 * Return the Design Capacity in µAh
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_dcap(struct bq27xxx_device_info *di)
{
	int dcap;

	dcap = bq27xxx_read(di, BQ27XXX_REG_DCAP, false);

	if (dcap < 0) {
		dev_dbg(di->dev, "error reading initial last measured discharge\n");
		return dcap;
	}

	if (di->chip == BQ27000 || di->chip == BQ27010)
		dcap *= BQ27XXX_CURRENT_CONSTANT / BQ27XXX_RS;
	else
		dcap *= 1000;

	return dcap;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_energy(struct bq27xxx_device_info *di)
{
	int ae;

	ae = bq27xxx_read(di, BQ27XXX_REG_AE, false);
	if (ae < 0) {
		dev_dbg(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27000 || di->chip == BQ27010)
		ae *= BQ27XXX_POWER_CONSTANT / BQ27XXX_RS;
	else
		ae *= 1000;

	return ae;
}

/*
 * Return the battery temperature in tenths of degree Kelvin
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_temperature(struct bq27xxx_device_info *di)
{
	int temp;

	temp = bq27xxx_read(di, BQ27XXX_REG_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}

	if (di->chip == BQ27000 || di->chip == BQ27010)
		temp = 5 * temp / 2;

	return temp;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_cyct(struct bq27xxx_device_info *di)
{
	int cyct;

	cyct = bq27xxx_read(di, BQ27XXX_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27xxx_battery_read_time(struct bq27xxx_device_info *di, u8 reg)
{
	int tval;

	tval = bq27xxx_read(di, reg, false);
	if (tval < 0) {
		dev_dbg(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

/*
 * Read an average power register.
 * Return < 0 if something fails.
 */
static int bq27xxx_battery_read_pwr_avg(struct bq27xxx_device_info *di)
{
	int tval;

	tval = bq27xxx_read(di, BQ27XXX_REG_AP, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading average power register  %02x: %d\n",
			BQ27XXX_REG_AP, tval);
		return tval;
	}

	if (di->chip == BQ27000 || di->chip == BQ27010)
		return (tval * BQ27XXX_POWER_CONSTANT) / BQ27XXX_RS;
	else
		return tval;
}

/*
 * Returns true if a battery over temperature condition is detected
 */
static bool bq27xxx_battery_overtemp(struct bq27xxx_device_info *di, u16 flags)
{
	if (di->chip == BQ27500 || di->chip == BQ27541 || di->chip == BQ27545)
		return flags & (BQ27XXX_FLAG_OTC | BQ27XXX_FLAG_OTD);
	if (di->chip == BQ27530 || di->chip == BQ27421)
		return flags & BQ27XXX_FLAG_OT;

	return false;
}

/*
 * Returns true if a battery under temperature condition is detected
 */
static bool bq27xxx_battery_undertemp(struct bq27xxx_device_info *di, u16 flags)
{
	if (di->chip == BQ27530 || di->chip == BQ27421)
		return flags & BQ27XXX_FLAG_UT;

	return false;
}

/*
 * Returns true if a low state of charge condition is detected
 */
static bool bq27xxx_battery_dead(struct bq27xxx_device_info *di, u16 flags)
{
	if (di->chip == BQ27000 || di->chip == BQ27010)
		return flags & (BQ27000_FLAG_EDV1 | BQ27000_FLAG_EDVF);
	else
		return flags & (BQ27XXX_FLAG_SOC1 | BQ27XXX_FLAG_SOCF);
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
static int bq27xxx_battery_read_health(struct bq27xxx_device_info *di)
{
	int flags;

	flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
	if (flags < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", flags);
		return flags;
	}

	/* Unlikely but important to return first */
	if (unlikely(bq27xxx_battery_overtemp(di, flags)))
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (unlikely(bq27xxx_battery_undertemp(di, flags)))
		return POWER_SUPPLY_HEALTH_COLD;
	if (unlikely(bq27xxx_battery_dead(di, flags)))
		return POWER_SUPPLY_HEALTH_DEAD;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static void dump_bq27xxx_regs(struct bq27xxx_device_info *di)
{
	u16 control = bq27xxx_read(di, BQ27XXX_REG_CTRL, false);
	u16 temp = bq27xxx_read(di, BQ27XXX_REG_TEMP, false);
	u16 volt = bq27xxx_read(di, BQ27XXX_REG_VOLT, false);
	u16 flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
	u16 fac = bq27xxx_read(di, BQ27XXX_REG_FAC, false);
	u16 rmc = bq27xxx_read(di, BQ27XXX_REG_RMC, false);
	u16 fcc = bq27xxx_read(di, BQ27XXX_REG_FCC, false);
	u16 ai = bq27xxx_read(di, BQ27XXX_REG_AI, false);
	u16 soc = bq27xxx_read(di, BQ27XXX_REG_SOC, false);
	u16 rcu = bq27xxx_read(di, BQ27XXX_REG_RCU, false);
	u16 fccu = bq27xxx_read(di, BQ27XXX_REG_FCCU, false);
	u16 socu = bq27xxx_read(di, BQ27XXX_REG_SOCU, false);
	u16 fccf = bq27xxx_read(di, BQ27XXX_REG_FCCF, false);

	dev_warn(di->dev, "Temperature:%u", temp);
	dev_warn(di->dev, "Flags:0x%04x\n", flags);
	dev_warn(di->dev, "StateOfCharge:%u\n", soc);
	dev_warn(di->dev, "RemainingCapacityUnfiltered:%u\n", rcu);
	dev_warn(di->dev, "FullChargeCapacityUnfiltered:%u\n", fccu);
	dev_warn(di->dev, "StateOfChargeUnfiltered:%u\n", socu);
	dev_warn(di->dev, "Control:0x%04x\n", control);
	dev_warn(di->dev, "Voltage:%u\n", volt);
	dev_warn(di->dev, "FullAvailableCapacity:%u\n", fac);
	dev_warn(di->dev, "RemainingCapacity:%u\n", rmc);
	dev_warn(di->dev, "FullChargeCapacity:%u\n", fcc);
	dev_warn(di->dev, "AverageCurrent:%d\n", (int)((s16)ai));
	dev_warn(di->dev, "FullChargeCapacityFiltered:%u\n", fccf);
}


static void bq27xxx_battery_update(struct bq27xxx_device_info *di)
{
	struct bq27xxx_reg_cache cache = {0, };
	bool has_ci_flag = di->chip == BQ27000 || di->chip == BQ27010;
	bool has_singe_flag = di->chip == BQ27000 || di->chip == BQ27010;
	static struct timeval cur = {0, };
	static struct timeval last = {0, };
	static kal_uint32 timer_counter;

	cache.flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, has_singe_flag);
	if ((cache.flags & 0xff) == 0xff)
		cache.flags = -1; /* read error */
	if (cache.flags >= 0) {
		cache.temperature = bq27xxx_battery_read_temperature(di);
		if (has_ci_flag && (cache.flags & BQ27000_FLAG_CI)) {
			dev_info(di->dev, "battery is not calibrated! ignoring capacity values\n");
			cache.capacity = -ENODATA;
			cache.energy = -ENODATA;
			cache.time_to_empty = -ENODATA;
			cache.time_to_empty_avg = -ENODATA;
			cache.time_to_full = -ENODATA;
			cache.charge_full = -ENODATA;
			cache.health = -ENODATA;
		} else {
			if (di->regs[BQ27XXX_REG_TTE] != INVALID_REG_ADDR)
				cache.time_to_empty = bq27xxx_battery_read_time(di, BQ27XXX_REG_TTE);
			if (di->regs[BQ27XXX_REG_TTECP] != INVALID_REG_ADDR)
				cache.time_to_empty_avg = bq27xxx_battery_read_time(di, BQ27XXX_REG_TTECP);
			if (di->regs[BQ27XXX_REG_TTF] != INVALID_REG_ADDR)
				cache.time_to_full = bq27xxx_battery_read_time(di, BQ27XXX_REG_TTF);
			cache.charge_full = bq27xxx_battery_read_fcc(di);
			cache.capacity = bq27xxx_battery_read_soc(di);
			if (di->regs[BQ27XXX_REG_AE] != INVALID_REG_ADDR)
				cache.energy = bq27xxx_battery_read_energy(di);
			cache.health = bq27xxx_battery_read_health(di);
		}
		if (di->regs[BQ27XXX_REG_CYCT] != INVALID_REG_ADDR)
			cache.cycle_count = bq27xxx_battery_read_cyct(di);
		if (di->regs[BQ27XXX_REG_AP] != INVALID_REG_ADDR)
			cache.power_avg = bq27xxx_battery_read_pwr_avg(di);

		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0)
			di->charge_design_full = bq27xxx_battery_read_dcap(di);
	}

	dev_warn(di->dev, "cache.capacity:%d, fake_capatity:%d, cache.temperature:%d, cache.flags:%08x,"
		              "BMT_status.bat_in_recharging_state:%d, BMT_status.bat_full:%d,"
			     "BMT_status.ICharging:%d, timer_counter:%d\n",
                      cache.capacity, di->fake_capacity, cache.temperature, cache.flags,
                      BMT_status.bat_in_recharging_state,
		      BMT_status.bat_full,BMT_status.ICharging, timer_counter);

	do_gettimeofday(&cur);

	/* Sync fake capacity to real capacity */
	dev_info(di->dev, "cur.tv_sec:%ld last.tv_sec:%ld\n", cur.tv_sec, last.tv_sec);
	if ((cur.tv_sec - last.tv_sec > SPM_TIMEOUT) && (last.tv_sec != 0)) {
		di->fake_capacity = cache.capacity;
		timer_counter = 0;
	}
	else {
		if (cache.capacity < di->fake_capacity) {
			cache.capacity = di->fake_capacity;
			if (!BMT_status.charger_exist && timer_counter == 2) {
				di->fake_capacity--;
				timer_counter = 0;
			}
			else {
				if (BMT_status.charger_exist)
					timer_counter = 0;
				else
					timer_counter ++;
			}
		}
		else {
			timer_counter = 0;
			di->fake_capacity = cache.capacity;
		}
	}
	last.tv_sec = cur.tv_sec;

	dump_bq27xxx_regs(di);

	mCapacity = cache.capacity;
	if (di->cache.capacity != cache.capacity) {
	    if ((BMT_status.charger_exist && (di->cache.capacity > cache.capacity)) ||
		(!BMT_status.charger_exist && (di->cache.capacity < cache.capacity)) )
		power_supply_changed(di->bat);
	}

	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0)
		di->cache = cache;

	di->last_update = jiffies;
}

static int control_cmd_wr(struct bq27xxx_device_info *di, u16 cmd)
{
	dev_dbg(di->dev, "%s: cmd - %04x\n", __func__, cmd);

	return di->bus.write(di, BQ27XXX_REG_CTRL, cmd, false);
}

static int control_cmd_read(struct bq27xxx_device_info *di, u16 cmd)
{
	dev_dbg(di->dev, "%s: cmd - %04x\n", __func__, cmd);

	di->bus.write(di, BQ27XXX_REG_CTRL, cmd, false);

	msleep(5);

	return di->bus.read(di, BQ27XXX_REG_CTRL, false);
}


static void copy_to_dm_buf_big_endian(struct bq27xxx_device_info *di,
	u8 *buf, u8 offset, u8 sz, u32 val)
{
	dev_dbg(di->dev, "%s: offset %d sz %d val %d\n",
		__func__, offset, sz, val);

	switch (sz) {
	case 1:
		buf[offset] = (u8) val;
		break;
	case 2:
		put_unaligned_be16((u16) val, &buf[offset]);
		break;
	case 4:
		put_unaligned_be32(val, &buf[offset]);
		break;
	default:
		dev_err(di->dev, "%s: bad size for dm parameter - %d",
			__func__, sz);
		break;
	}
}

#define SEAL_UNSEAL_POLLING_RETRY_LIMIT	1000

static inline int sealed(struct bq27xxx_device_info *di)
{
	return control_cmd_read(di, CONTROL_STATUS_SUBCMD) & (1 << 13);
}

static int unseal(struct bq27xxx_device_info *di, u32 key)
{
	int i = 0;

	dev_dbg(di->dev, "%s: key - %08x\n", __func__, key);

	if (!sealed(di))
		goto out;

	di->bus.write(di, BQ27XXX_REG_CTRL, key & 0xFFFF, false);
	msleep(5);
	di->bus.write(di, BQ27XXX_REG_CTRL, (key & 0xFFFF0000) >> 16, false);
	msleep(5);

	while (i < SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		i++;
		if (!sealed(di))
			break;
		msleep(10);
	}

out:
	if (i == SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed\n", __func__);
		return 0;
	} else {
		return 1;
	}
}

static int seal(struct bq27xxx_device_info *di)
{
	int i = 0;
	int is_sealed;

	dev_dbg(di->dev, "%s:\n", __func__);

	is_sealed = sealed(di);
	if (is_sealed)
		return is_sealed;

	di->bus.write(di, BQ27XXX_REG_CTRL, SEAL_SUBCMD, false);

	while (i < SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
		i++;
		is_sealed = sealed(di);
		if (is_sealed)
			break;
		msleep(10);
	}

	if (!is_sealed)
		dev_err(di->dev, "%s: failed\n", __func__);

	return is_sealed;
}


#define CFG_UPDATE_POLLING_RETRY_LIMIT 50
static int enter_cfg_update_mode(struct bq27xxx_device_info *di)
{
	int i = 0;
	u16 flags;

	dev_dbg(di->dev, "%s:\n", __func__);

	if (!unseal(di, BQ274XX_UNSEAL_KEY))
		return 0;

	control_cmd_wr(di, SET_CFGUPDATE_SUBCMD);
	msleep(5);

	while (i < CFG_UPDATE_POLLING_RETRY_LIMIT) {
		i++;
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (flags & (1 << 4))
			break;
		msleep(100);
	}

	if (i == CFG_UPDATE_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed %04x\n", __func__, flags);
		return 0;
	}

	return 1;
}

static int exit_cfg_update_mode(struct bq27xxx_device_info *di)
{
	int i = 0;
	u16 flags;

	dev_dbg(di->dev, "%s:\n", __func__);

	control_cmd_wr(di, BQ274XX_EXIT_CFGUPDATE);

	while (i < CFG_UPDATE_POLLING_RETRY_LIMIT) {
		i++;
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (!(flags & (1 << 4)))
			break;
		msleep(100);
	}

	if (i == CFG_UPDATE_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: failed %04x\n", __func__, flags);
		return 0;
	}

	if (seal(di))
		return 1;
	else
		return 0;
}

static u8 checksum(u8 *data)
{
	u16 sum = 0;
	int i;

	for (i = 0; i < 32; i++)
		sum += data[i];

	sum &= 0xFF;

	return 0xFF - sum;
}

static int update_dm_block(struct bq27xxx_device_info *di, u8 subclass,
	u8 offset, u8 *data)
{
	u8 buf[32];
	u8 cksum;
	u8 blk_offset = offset >> 5;

	dev_warn(di->dev, "%s: subclass %d offset %d\n",
		__func__, subclass, offset);

	di->bus.write(di, BLOCK_DATA_CONTROL, 0, true);
	msleep(5);

	di->bus.write(di, BLOCK_DATA_CLASS, subclass, true);
	msleep(5);

	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);

	di->bus.blk_write(di, BLOCK_DATA, data, 32);
	msleep(5);

	cksum = checksum(data);
	di->bus.write(di, BLOCK_DATA_CHECKSUM, cksum, true);
	msleep(5);

	/* Read back and compare to make sure write is successful */
	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);
	di->bus.blk_read(di, BLOCK_DATA, buf, 32);

	if (memcmp(data, buf, 32)) {
		dev_err(di->dev, "%s: error updating subclass %d offset %d\n",
			__func__, subclass, offset);
		return 0;
	} else {
		dev_warn(di->dev, "%s:  Update successful subclass %d offset %d\n",
			__func__, subclass, offset);
		return 1;
	}
}

static int read_dm_block(struct bq27xxx_device_info *di, u8 subclass,
	u8 offset, u8 *data)
{
	u8 cksum_calc, cksum;
	u8 blk_offset = offset >> 5;

	dev_dbg(di->dev, "%s: subclass %d offset %d\n",
		__func__, subclass, offset);

	di->bus.write(di, BLOCK_DATA_CONTROL, 0, true);
	msleep(5);

	di->bus.write(di, BLOCK_DATA_CLASS, subclass, true);
	msleep(5);

	di->bus.write(di, DATA_BLOCK, blk_offset, true);
	msleep(5);

	di->bus.blk_read(di, BLOCK_DATA, data, 32);

	cksum_calc = checksum(data);
	cksum = di->bus.read(di, BLOCK_DATA_CHECKSUM, true);
	if (cksum != cksum_calc) {
		dev_err(di->dev, "%s: error reading subclass %d offset %d\n",
			__func__, subclass, offset);
		return 0;
	}

	return 1;
}


static int rom_mode_gauge_init_completed(struct bq27xxx_device_info *di)
{
	dev_dbg(di->dev, "%s:\n", __func__);

	return control_cmd_read(di, CONTROL_STATUS_SUBCMD) &
		BQ274XX_CTRL_STATUS_INITCOMP;
}

static bool rom_mode_gauge_dm_initialized(struct bq27xxx_device_info *di)
{
	u16 flags;

	flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);

	dev_dbg(di->dev, "%s: flags - 0x%04x\n", __func__, flags);

	if (flags & BQ274XX_FLAG_ITPOR)
		return false;
	else
		return true;
}


#define INITCOMP_TIMEOUT_MS		10000
static void rom_mode_gauge_dm_init(struct bq27xxx_device_info *di)
{
	int i;
	int timeout = INITCOMP_TIMEOUT_MS;
	u8 subclass, offset;
	u32 blk_number;
	u32 blk_number_prev = 0;
	u8 buf[32];
	bool buf_valid = false;
	struct dm_reg *dm_reg;

	dev_dbg(di->dev, "%s:\n", __func__);

	while (!rom_mode_gauge_init_completed(di) && timeout > 0) {
		msleep(100);
		timeout -= 100;
	}

	if (timeout <= 0) {
		dev_err(di->dev, "%s: INITCOMP not set after %d seconds\n",
			__func__, INITCOMP_TIMEOUT_MS/100);
		return;
	}

	if (!di->dm_regs || !di->dm_regs_count) {
		dev_err(di->dev, "%s: Data not available for DM initialization\n",
			__func__);
		return;
	}

	dev_warn(di->dev, "start rom_mode_gauge_dm_init\n");
	enter_cfg_update_mode(di);
	for (i = 0; i < di->dm_regs_count; i++) {
		dm_reg = &di->dm_regs[i];
		subclass = dm_reg->subclass;
		offset = dm_reg->offset;

		/*
		 * Create a composite block number to see if the subsequent
		 * register also belongs to the same 32 btye block in the DM
		 */
		blk_number = subclass << 8;
		blk_number |= offset >> 5;

		if (blk_number == blk_number_prev) {
			copy_to_dm_buf_big_endian(di, buf, offset,
				dm_reg->len, dm_reg->data);
		} else {

			if (buf_valid)
				update_dm_block(di, blk_number_prev >> 8,
					(blk_number_prev << 5) & 0xFF , buf);
			else
				buf_valid = true;

			read_dm_block(di, dm_reg->subclass, dm_reg->offset,
				buf);
			copy_to_dm_buf_big_endian(di, buf, offset - ((blk_number << 5) & 0xFF),
				dm_reg->len, dm_reg->data);
		}
		blk_number_prev = blk_number;
	}

	/* Last buffer to be written */
	if (buf_valid)
		update_dm_block(di, subclass, offset, buf);

	exit_cfg_update_mode(di);
}

struct subclass_reg {
	u8 subclass;
	u8 offset;
};

static struct subclass_reg subcla_regs[] = {
	{64, 0},
	{64, 2},
	{80, 78},   /* TermV Valid t 10s */
	{82, 0},
	{82, 2},
	{82, 3},   /* Reserve capacity */
	{82, 8},
	{82, 10},	/* Design Capacity */
	{82, 16},	/* Terminate Voltage */
	{82, 22},	/* Taper rate */
	{82, 29},
	{89, 0},
	{89, 2},
	{89, 12},
	{89, 20},
	{89, 28},
};

static void rom_mode_print_subclass(struct bq27xxx_device_info *di)
{
	static int count = 0;
	int i;
	int timeout = INITCOMP_TIMEOUT_MS;
	u8 subclass, offset;
	u8 buf[32];
	struct subclass_reg *sb_reg;

	count++;
	if(count%10)
		return;

	dev_dbg(di->dev, "%s:\n", __func__);

	while (!rom_mode_gauge_init_completed(di) && timeout > 0) {
		msleep(100);
		timeout -= 100;
	}

	if (timeout <= 0) {
		dev_err(di->dev, "%s: INITCOMP not set after %d seconds\n",
			__func__, INITCOMP_TIMEOUT_MS/100);
		return;
	}

	enter_cfg_update_mode(di);
	for (i = 0; i < ARRAY_SIZE(subcla_regs); i++) {
		sb_reg = &subcla_regs[i];
		subclass = sb_reg->subclass;
		offset = sb_reg->offset;
		read_dm_block(di, sb_reg->subclass, sb_reg->offset,
				buf);
	}
	exit_cfg_update_mode(di);
}


static int get_terminate_voltage(struct bq27xxx_device_info *di)
{
	u8 buf[32] = {0};
	u8 subclass = 82;
	u8 offset = 0;

	if (!unseal(di, BQ274XX_UNSEAL_KEY))
			return 0;

	read_dm_block(di, subclass, offset, buf);

	seal(di);

	return ((buf[16] & 0xFF)<<8)+(buf[17] & 0xFF);
}

static void bq27xxx_battery_poll(struct work_struct *work)
{
	struct bq27xxx_device_info *di =
			container_of(work, struct bq27xxx_device_info,
				     work.work);

	if ((di->chip == BQ27421) && !rom_mode_gauge_dm_initialized(di)) {
		rom_mode_gauge_dm_init(di);
	}

	rom_mode_print_subclass(di);

	bq27xxx_battery_update(di);

	if (poll_interval > 0) {
		/* The timer does not have to be accurate. */
		set_timer_slack(&di->work.timer, poll_interval * HZ / 4);
		schedule_delayed_work(&di->work, poll_interval * HZ);
	}
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27xxx_battery_current(struct bq27xxx_device_info *di,
				   union power_supply_propval *val)
{
	int curr;
	int flags;

	curr = bq27xxx_read(di, BQ27XXX_REG_AI, false);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
		return curr;
	}

	if (di->chip == BQ27000 || di->chip == BQ27010) {
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * BQ27XXX_CURRENT_CONSTANT / BQ27XXX_RS;
	} else {
		/* Other gauges return signed value */
		val->intval = (int)((s16)curr) * 1000;
	}

	return 0;
}

static int bq27xxx_battery_status(struct bq27xxx_device_info *di,
				  union power_supply_propval *val)
{
	int status;

	if (di->chip == BQ27000 || di->chip == BQ27010) {
		if (di->cache.flags & BQ27000_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		struct bq27xxx_reg_cache cache = {0, };
		bool has_singe_flag = di->chip == BQ27000 || di->chip == BQ27010;

		cache.flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, has_singe_flag);
		if ((cache.flags & BQ27XXX_FLAG_FC) && BMT_status.charger_exist)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (cache.flags & BQ27XXX_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	}

	val->intval = status;

	return 0;
}

static int bq27xxx_battery_capacity_level(struct bq27xxx_device_info *di,
					  union power_supply_propval *val)
{
	int level;

	if (di->chip == BQ27000 || di->chip == BQ27010) {
		if (di->cache.flags & BQ27000_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27000_FLAG_EDV1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27000_FLAG_EDVF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	} else {
		if (di->cache.flags & BQ27XXX_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27XXX_FLAG_SOC1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27XXX_FLAG_SOCF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}

	val->intval = level;

	return 0;
}

/*
 * Return the battery Voltage in millivolts
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_voltage(struct bq27xxx_device_info *di,
				   union power_supply_propval *val)
{
	int volt;

	volt = bq27xxx_read(di, BQ27XXX_REG_VOLT, false);
	if (volt < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return volt;
	}

	val->intval = volt * 1000;

	return 0;
}

static int bq27xxx_simple_value(int value,
				union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

int bq27xxx_get_battery_percentage(void)
{
    return mCapacity;
}
EXPORT_SYMBOL(bq27xxx_get_battery_percentage);

static int bq27xxx_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27xxx_device_info *di = power_supply_get_drvdata(psy);

	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27xxx_battery_poll(&di->work.work);
	}
	mutex_unlock(&di->lock);

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27xxx_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27xxx_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27xxx_battery_current(di, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if(BMT_status.bat_in_recharging_state || BMT_status.bat_full) {
			val->intval = 100;
			di->fake_capacity = 100;
		}
		else
			ret = bq27xxx_simple_value(di->cache.capacity, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27xxx_battery_capacity_level(di, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq27xxx_simple_value(di->cache.temperature, val);
		if (ret == 0)
			val->intval -= 2731; /* convert decidegree k to c */
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27xxx_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27xxx_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27xxx_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27xxx_simple_value(bq27xxx_battery_read_nac(di), val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27xxx_simple_value(di->cache.charge_full, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27xxx_simple_value(di->charge_design_full, val);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27xxx_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27xxx_simple_value(di->cache.energy, val);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		ret = bq27xxx_simple_value(di->cache.power_avg, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27xxx_simple_value(di->cache.health, val);
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ27XXX_MANUFACTURER;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27xxx_external_power_changed(struct power_supply *psy)
{
	struct bq27xxx_device_info *di = power_supply_get_drvdata(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}

static int bq27xxx_soft_reset(struct bq27xxx_device_info *di)
{
	int i = 0;
	u16 flags;

	dev_warn(di->dev, "%s: Enter. \n", __func__);

	dev_warn(di->dev, "%s: Enter cfg update mode\n", __func__);
	enter_cfg_update_mode(di);

	dev_warn(di->dev, "%s: cmd - %04x\n", __func__, BQ274XX_SOFT_RESET);
	control_cmd_wr(di, BQ274XX_SOFT_RESET);

	while (i < CFG_UPDATE_POLLING_RETRY_LIMIT) {
		i++;
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (!(flags & (1 << 4)))
			break;
		msleep(100);
	}

	if (i == CFG_UPDATE_POLLING_RETRY_LIMIT) {
		dev_err(di->dev, "%s: Exit. soft reset failed %04x!\n", __func__, flags);
		return 0;
	}

	if (seal(di)) {
		dev_warn(di->dev, "%s: Exit. soft reset successfully! \n", __func__);
		return 1;
	}
	else {
		dev_warn(di->dev, "%s: Exit. soft reset seal failed!\n", __func__);
		return 0;
	}
}

static int bq27xxx_powersupply_init(struct bq27xxx_device_info *di,
				    const char *name)
{
	int ret;
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = { .drv_data = di, };
	int terminate_voltage = 0;
	int socu = 0;
	int fcc = 0;

	psy_desc = devm_kzalloc(di->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;

	psy_desc->name = name;
	psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	psy_desc->properties = bq27xxx_battery_props[di->chip].props;
	psy_desc->num_properties = bq27xxx_battery_props[di->chip].size;
	psy_desc->get_property = bq27xxx_battery_get_property;
	psy_desc->external_power_changed = bq27xxx_external_power_changed;

	INIT_DELAYED_WORK(&di->work, bq27xxx_battery_poll);
	mutex_init(&di->lock);

	di->bat = power_supply_register_no_ws(di->dev, psy_desc, &psy_cfg);
	if (IS_ERR(di->bat)) {
		ret = PTR_ERR(di->bat);
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	bq27xxx_battery_update(di);

	if (di->chip == BQ27421) {
		/*if socu is far greater than soc, align soc with socu. */
		socu = bq27xxx_battery_read_socu(di);
		fcc = bq27xxx_battery_read_fcc(di)/1000;

		dev_warn(di->dev, "%s: soc: %d; socu: %d; fcc: %d\n",
				__func__, di->cache.capacity, socu, fcc);
		if ( ((abs(di->cache.capacity - socu) > 9) && (socu >= 0)) || (fcc > 800)  ) {
			bq27xxx_soft_reset(di);
			rom_mode_gauge_dm_init(di);
		}
		else {
			terminate_voltage = get_terminate_voltage(di);
			dev_warn(di->dev, "%s: terminate_voltage:%d\n",
				__func__, terminate_voltage);

			if (3200 == terminate_voltage || 3400 == terminate_voltage) {
				rom_mode_gauge_dm_init(di);
			}
		}
	}

	return 0;
}

static void bq27xxx_powersupply_unregister(struct bq27xxx_device_info *di)
{
	/*
	 * power_supply_unregister call bq27xxx_battery_get_property which
	 * call bq27xxx_battery_poll.
	 * Make sure that bq27xxx_battery_poll will not call
	 * schedule_delayed_work again after unregister (which cause OOPS).
	 */
	poll_interval = 0;

	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(di->bat);

	mutex_destroy(&di->lock);
}

/* i2c specific code */
#ifdef CONFIG_BATTERY_BQ27XXX_I2C

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static irqreturn_t bq27xxx_battery_irq_handler_thread(int irq, void *data)
{
	struct bq27xxx_device_info *di = data;

	bq27xxx_battery_update(di);

	return IRQ_HANDLED;
}

static int bq27xxx_battery_i2c_read(struct bq27xxx_device_info *di, u8 reg,
				    bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	memset(msg, 0, sizeof(msg));
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];

	return ret;
}

static int bq27xxx_write_i2c(struct bq27xxx_device_info *di, u8 reg, int value, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg;
	unsigned char data[4];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	memset(&msg, 0, sizeof(msg));
	data[0] = reg;
	if (single) {
		data[1] = (unsigned char)value;
		msg.len = 2;
	} else {
		put_unaligned_le16(value, &data[1]);
		msg.len = 3;
	}

	msg.buf = data;
	msg.addr = client->addr;
	msg.flags = 0;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int bq27xxx_read_i2c_blk(struct bq27xxx_device_info *di, u8 reg,
	u8 *data, u8 len)
{
	int i;
	for (i = 0; i < len; i++)
		data[i] = di->bus.read(di, reg+i, true);

	return 0;
}

static int bq27xxx_write_i2c_blk(struct bq27xxx_device_info *di, u8 reg,
	u8 *data, u8 sz)
{
	int i;

	for (i = 0; i < sz; i++)
		di->bus.write(di, reg+i, data[i], true);

	return 0;
}


static int bq27xxx_battery_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	char *name;
	struct bq27xxx_device_info *di;
	int num;
	int retval = 0;

	/* Get new ID for the new battery device */
	mutex_lock(&battery_mutex);
	num = idr_alloc(&battery_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&battery_mutex);
	if (num < 0)
		return num;

	name = devm_kasprintf(&client->dev, GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		retval = -ENOMEM;
		goto batt_failed;
	}

	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto batt_failed;
	}

	di->id = num;
	di->dev = &client->dev;
	di->chip = id->driver_data;
	di->bus.read = &bq27xxx_battery_i2c_read;
	di->bus.write = &bq27xxx_write_i2c;
	di->bus.blk_read = bq27xxx_read_i2c_blk;
	di->bus.blk_write = bq27xxx_write_i2c_blk;
	di->regs = bq27xxx_regs[di->chip];
	di->dm_regs = bq274xx_dm_regs;
	di->dm_regs_count = ARRAY_SIZE(bq274xx_dm_regs);

	retval = bq27xxx_powersupply_init(di, name);
	if (retval)
		goto batt_failed;

	/* Schedule a polling after about 1 min */
	schedule_delayed_work(&di->work, 60 * HZ);

	i2c_set_clientdata(client, di);

	if (client->irq) {
		retval = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, bq27xxx_battery_irq_handler_thread,
				IRQF_ONESHOT,
				name, di);
		if (retval) {
			dev_err(&client->dev,
				"Unable to register IRQ %d error %d\n",
				client->irq, retval);
			return retval;
		}
	}

	return 0;

batt_failed:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27xxx_battery_i2c_remove(struct i2c_client *client)
{
	struct bq27xxx_device_info *di = i2c_get_clientdata(client);

	bq27xxx_powersupply_unregister(di);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	return 0;
}

static struct i2c_board_info __initdata i2c_bq27421_boardinfo =
{
	I2C_BOARD_INFO("bq27421", (0x55))
};

static const struct i2c_device_id bq27xxx_id[] = {
	{ "bq27200", BQ27000 },
	{ "bq27210", BQ27010 },
	{ "bq27500", BQ27500 },
	{ "bq27510", BQ27500 },
	{ "bq27520", BQ27500 },
	{ "bq27530", BQ27530 },
	{ "bq27531", BQ27530 },
	{ "bq27541", BQ27541 },
	{ "bq27542", BQ27541 },
	{ "bq27546", BQ27541 },
	{ "bq27742", BQ27541 },
	{ "bq27545", BQ27545 },
	{ "bq27421", BQ27421 },
	{ "bq27425", BQ27421 },
	{ "bq27441", BQ27421 },
	{ "bq27621", BQ27421 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27xxx_id);

static struct i2c_driver bq27xxx_battery_i2c_driver = {
	.driver = {
		.name = "bq27xxx-battery",
	},
	.probe = bq27xxx_battery_i2c_probe,
	.remove = bq27xxx_battery_i2c_remove,
	.id_table = bq27xxx_id,
};

static inline int bq27xxx_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27xxx_battery_i2c_driver);

	if (ret)
		pr_err("Unable to register BQ27xxx i2c driver\n");

	return ret;
}

static inline void bq27xxx_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27xxx_battery_i2c_driver);
}

#else

static inline int bq27xxx_battery_i2c_init(void) { return 0; }
static inline void bq27xxx_battery_i2c_exit(void) {};

#endif

/* platform specific code */
#ifdef CONFIG_BATTERY_BQ27XXX_PLATFORM

static int bq27xxx_battery_platform_read(struct bq27xxx_device_info *di, u8 reg,
					 bool single)
{
	struct device *dev = di->dev;
	struct bq27xxx_platform_data *pdata = dev->platform_data;
	unsigned int timeout = 3;
	int upper, lower;
	int temp;

	if (!single) {
		/* Make sure the value has not changed in between reading the
		 * lower and the upper part */
		upper = pdata->read(dev, reg + 1);
		do {
			temp = upper;
			if (upper < 0)
				return upper;

			lower = pdata->read(dev, reg);
			if (lower < 0)
				return lower;

			upper = pdata->read(dev, reg + 1);
		} while (temp != upper && --timeout);

		if (timeout == 0)
			return -EIO;

		return (upper << 8) | lower;
	}

	return pdata->read(dev, reg);
}

static int bq27xxx_battery_platform_probe(struct platform_device *pdev)
{
	struct bq27xxx_device_info *di;
	struct bq27xxx_platform_data *pdata = pdev->dev.platform_data;
	const char *name;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform_data supplied\n");
		return -EINVAL;
	}

	if (!pdata->read) {
		dev_err(&pdev->dev, "no hdq read callback supplied\n");
		return -EINVAL;
	}

	if (!pdata->chip) {
		dev_err(&pdev->dev, "no device supplied\n");
		return -EINVAL;
	}

	di = devm_kzalloc(&pdev->dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->chip = pdata->chip;
	di->regs = bq27xxx_regs[di->chip];

	name = pdata->name ?: dev_name(&pdev->dev);
	di->bus.read = &bq27xxx_battery_platform_read;

	return bq27xxx_powersupply_init(di, name);
}

static int bq27xxx_battery_platform_remove(struct platform_device *pdev)
{
	struct bq27xxx_device_info *di = platform_get_drvdata(pdev);

	bq27xxx_powersupply_unregister(di);

	return 0;
}

static struct platform_driver bq27xxx_battery_platform_driver = {
	.probe	= bq27xxx_battery_platform_probe,
	.remove = bq27xxx_battery_platform_remove,
	.driver = {
		.name = "bq27000-battery",
	},
};

static inline int bq27xxx_battery_platform_init(void)
{
	int ret = platform_driver_register(&bq27xxx_battery_platform_driver);

	if (ret)
		pr_err("Unable to register BQ27xxx platform driver\n");

	return ret;
}

static inline void bq27xxx_battery_platform_exit(void)
{
	platform_driver_unregister(&bq27xxx_battery_platform_driver);
}

#else

static inline int bq27xxx_battery_platform_init(void) { return 0; }
static inline void bq27xxx_battery_platform_exit(void) {};

#endif

/*
 * Module stuff
 */

static int __init bq27xxx_battery_init(void)
{
	int ret;

	i2c_register_board_info(1, &i2c_bq27421_boardinfo, 1);
	ret = bq27xxx_battery_i2c_init();

	if (ret)
		return ret;

	ret = bq27xxx_battery_platform_init();
	if (ret)
		bq27xxx_battery_i2c_exit();

	return ret;
}
module_init(bq27xxx_battery_init);

static void __exit bq27xxx_battery_exit(void)
{
	bq27xxx_battery_platform_exit();
	bq27xxx_battery_i2c_exit();
}
module_exit(bq27xxx_battery_exit);

#ifdef CONFIG_BATTERY_BQ27XXX_PLATFORM
MODULE_ALIAS("platform:bq27000-battery");
#endif

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27xxx battery monitor driver");
MODULE_LICENSE("GPL");
