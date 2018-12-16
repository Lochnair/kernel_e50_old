/*
 * Copyright (C) 2013 Realtek Semiconductor Corp.
 * All Rights Reserved.
 *
 * Unless you and Realtek execute a separate written software license
 * agreement governing use of this software, this software is licensed
 * to you under the terms of the GNU General Public License version 2,
 * available at https://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 * Purpose : RTL8367C switch low-level function for access register
 * Feature : SMI related functions
 *
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/kernel.h>
#include <linux/sched.h>

#include <rtk_types.h>
#include <smi.h>
#include "rtk_error.h"
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <gsw_mt7620.h>
//#include "raeth_reg.h"

static DEFINE_MUTEX(mii_lock);
EXPORT_SYMBOL(mii_lock);

#if defined(MDC_MDIO_OPERATION)
/*******************************************************************************/
/*  MDC/MDIO porting                                                           */
/*******************************************************************************/
/* define the PHY ID currently used */
/* carlos */
#if 0
#define MDC_MDIO_PHY_ID     0  /* PHY ID 0 or 29 */
#else
#define MDC_MDIO_PHY_ID     29  /* PHY ID 0 or 29 */
#endif

/* MDC/MDIO, redefine/implement the following Macro */ /*carlos*/
#if 0
#define MDC_MDIO_WRITE(preamableLength, phyID, regID, data)
#define MDC_MDIO_READ(preamableLength, phyID, regID, pData)
#else
#define u32      unsigned int
//extern u32 mii_mgr_read(u32 phy_addr, u32 phy_register, u32 *read_data);
//extern u32 mii_mgr_write(u32 phy_addr, u32 phy_register, u32 write_data);
extern struct mt7620_gsw *gsw_mt7621;
extern void __iomem *fe_base;
extern u32 _mt7620_mii_read(struct mt7620_gsw *gsw, int phy_addr, int phy_reg);
extern u32 _mt7620_mii_write(struct mt7620_gsw *gsw, u32 phy_addr, u32 phy_register, u32 write_data);

#define MDC_MDIO_WRITE(preamableLength, phyID, regID, data) mii_mgr_write(phyID, regID, data)
#define MDC_MDIO_READ(preamableLength, phyID, regID, pData) mii_mgr_read(phyID, regID, pData)
#endif

#elif defined(SPI_OPERATION)
/*******************************************************************************/
/*  SPI porting                                                                */
/*******************************************************************************/
/* SPI, redefine/implement the following Macro */
#define SPI_WRITE(data, length)
#define SPI_READ(pData, length)





#else
/*******************************************************************************/
/*  I2C porting                                                                */
/*******************************************************************************/
/* Define the GPIO ID for SCK & SDA */
rtk_uint32  smi_SCK = 1;    /* GPIO used for SMI Clock Generation */
rtk_uint32  smi_SDA = 2;    /* GPIO used for SMI Data signal */

/* I2C, redefine/implement the following Macro */
#define GPIO_DIRECTION_SET(gpioID, direction)
#define GPIO_DATA_SET(gpioID, data)
#define GPIO_DATA_GET(gpioID, pData)
#endif

//UBNT_Andrew 2018/05/02
static spinlock_t g_mdio_lock;
#define ESW_PHY_POLLING		0x0000
#define PHY_CONTROL_0		0x0004
#define MDIO_PHY_CONTROL_0	((fe_base + 0x10000) + PHY_CONTROL_0)
#define enable_mdio(x)

unsigned long flags;

void set_an_polling(u32 an_status)
{
	if (an_status == 1)
		*(unsigned long *)(ESW_PHY_POLLING) |= (1 << 31);
	else
		*(unsigned long *)(ESW_PHY_POLLING) &= ~(1 << 31);
}

u32 __mii_mgr_read(u32 phy_addr, u32 phy_register, u32 *read_data)
{
	u32 status = 0;
	u32 rc = 0;
	unsigned long t_start = jiffies;
	u32 data = 0;


	/* make sure previous read operation is complete */
	while (1) {
		/* 0 : Read/write operation complete */
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			break;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			pr_err("\n MDIO Read operation is ongoing !!\n");
			return rc;
		}
	}

	data =
	    (0x01 << 16) | (0x02 << 18) | (phy_addr << 20) | (phy_register <<
							      25);
	sys_reg_write(MDIO_PHY_CONTROL_0, data);
	sys_reg_write(MDIO_PHY_CONTROL_0, (data | (1 << 31)));

	/* make sure read operation is complete */
	t_start = jiffies;
	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			status = sys_reg_read(MDIO_PHY_CONTROL_0);
			*read_data = (u32)(status & 0x0000FFFF);

			return 1;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			pr_err
			    ("\n MDIO Read operation Time Out!!\n");
			return 0;
		}
	}
}

u32 __mii_mgr_write(u32 phy_addr, u32 phy_register, u32 write_data)
{
	unsigned long t_start = jiffies;
	u32 data;

	/* make sure previous write operation is complete */
	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			break;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			pr_err("\n MDIO Write operation ongoing\n");
			return 0;
		}
	}

	data =
	    (0x01 << 16) | (1 << 18) | (phy_addr << 20) | (phy_register << 25) |
	    write_data;
	sys_reg_write(MDIO_PHY_CONTROL_0, data);
	sys_reg_write(MDIO_PHY_CONTROL_0, (data | (1 << 31))); /*start*/
	/* pr_err("\n Set Command [0x%08X] to PHY !!\n",MDIO_PHY_CONTROL_0); */

	t_start = jiffies;

	/* make sure write operation is complete */
	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			return 1;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			pr_err("\n MDIO Write operation Time Out\n");
			return 0;
		}
	}
}


u32 mii_mgr_read(u32 phy_addr, u32 phy_register, u32 *read_data)
{
	#if 1
	u16 high, low;

	if(phy_addr == 31) {
		_mt7620_mii_write(gsw_mt7621, phy_addr, 0x1f, (phy_register >> 6) & 0x3ff);
		low = _mt7620_mii_read(gsw_mt7621, phy_addr, (phy_register >> 2) & 0xf);
		high = _mt7620_mii_read(gsw_mt7621, phy_addr, 0x10);

		*read_data = (high << 16) | (low & 0xffff);
	}
	else
		*read_data = _mt7620_mii_read(gsw_mt7621, phy_addr, phy_register);
		
	//if(*read_data == 0xffffffff)
	//	return 1;

	return 0;
	#else
	u32 low_word;
	u32 high_word;
	u32 an_status = 0;

	if (phy_addr == 31) {
		an_status = (*(unsigned long *)(ESW_PHY_POLLING) & (1 << 31));
		if (an_status)
			set_an_polling(0);
		if (__mii_mgr_write
		    (phy_addr, 0x1f, ((phy_register >> 6) & 0x3FF))) {
			if (__mii_mgr_read
			    (phy_addr, (phy_register >> 2) & 0xF, &low_word)) {
				if (__mii_mgr_read
				    (phy_addr, (0x1 << 4), &high_word)) {
					*read_data =
					    (high_word << 16) | (low_word &
								 0xFFFF);
					if (an_status)
						set_an_polling(1);
					return 1;
				}
			}
		}
		if (an_status)
			set_an_polling(1);
	} else {
		if (__mii_mgr_read(phy_addr, phy_register, read_data))
			return 1;
	}
	return 0;
	#endif
}
EXPORT_SYMBOL(mii_mgr_read);

u32 mii_mgr_write(u32 phy_addr, u32 phy_register, u32 write_data)
{
	#if 1
	int ret = 0;

	if(phy_addr == 31) {
		ret |= _mt7620_mii_write(gsw_mt7621, phy_addr, 0x1f, (phy_register >> 6) & 0x3ff);
		ret |= _mt7620_mii_write(gsw_mt7621, phy_addr, (phy_register >> 2) & 0xf,  write_data & 0xffff);
		ret |= _mt7620_mii_write(gsw_mt7621, phy_addr, 0x10, write_data >> 16);
	}
	else
		ret = _mt7620_mii_write(gsw_mt7621, phy_addr, phy_register, write_data);

	//if(ret != 0)
	//	return 1;
	ret = 0;

	return ret;
	#else
	u32 an_status = 0;

	if (phy_addr == 31) {
		an_status = (*(unsigned long *)(ESW_PHY_POLLING) & (1 << 31));
		if (an_status)
			set_an_polling(0);
		if (__mii_mgr_write
		    (phy_addr, 0x1f, (phy_register >> 6) & 0x3FF)) {
			if (__mii_mgr_write
			    (phy_addr, ((phy_register >> 2) & 0xF),
			     write_data & 0xFFFF)) {
				if (__mii_mgr_write
				    (phy_addr, (0x1 << 4), write_data >> 16)) {
					if (an_status)
						set_an_polling(1);
					return 1;
				}
			}
		}
		if (an_status)
			set_an_polling(1);
	} else {
		if (__mii_mgr_write(phy_addr, phy_register, write_data))
			return 1;
	}

	return 0;
	#endif
}
EXPORT_SYMBOL(mii_mgr_write);

u32 mii_mgr_cl45_set_address(u32 port_num, u32 dev_addr, u32 reg_addr)
{
	u32 rc = 0;
	unsigned long t_start = jiffies;
	u32 data = 0;

	enable_mdio(1);

	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			break;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			enable_mdio(0);
			pr_err("\n MDIO Read operation is ongoing !!\n");
			return rc;
		}
	}
	data =
	    (dev_addr << 25) | (port_num << 20) | (0x00 << 18) | (0x00 << 16) |
	    reg_addr;
	sys_reg_write(MDIO_PHY_CONTROL_0, data);
	sys_reg_write(MDIO_PHY_CONTROL_0, (data | (1 << 31)));

	t_start = jiffies;
	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			enable_mdio(0);
			return 1;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			enable_mdio(0);
			pr_err("\n MDIO Write operation Time Out\n");
			return 0;
		}
	}
}

u32 mii_mgr_read_cl45(u32 port_num, u32 dev_addr, u32 reg_addr, u32 *read_data)
{
	u32 status = 0;
	u32 rc = 0;
	unsigned long t_start = jiffies;
	u32 data = 0;

	/* set address first */
	mii_mgr_cl45_set_address(port_num, dev_addr, reg_addr);
	/* udelay(10); */

	enable_mdio(1);

	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			break;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			enable_mdio(0);
			pr_err("\n MDIO Read operation is ongoing !!\n");
			return rc;
		}
	}
	data =
	    (dev_addr << 25) | (port_num << 20) | (0x03 << 18) | (0x00 << 16) |
	    reg_addr;
	sys_reg_write(MDIO_PHY_CONTROL_0, data);
	sys_reg_write(MDIO_PHY_CONTROL_0, (data | (1 << 31)));
	t_start = jiffies;
	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			*read_data =
			    (sys_reg_read(MDIO_PHY_CONTROL_0) & 0x0000FFFF);
			enable_mdio(0);
			return 1;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			enable_mdio(0);
			pr_err
			    ("\n MDIO Read operation Time Out!!\n");
			return 0;
		}
		status = sys_reg_read(MDIO_PHY_CONTROL_0);
	}
}
EXPORT_SYMBOL(mii_mgr_read_cl45);

u32 mii_mgr_write_cl45(u32 port_num, u32 dev_addr, u32 reg_addr, u32 write_data)
{
	u32 rc = 0;
	unsigned long t_start = jiffies;
	u32 data = 0;

	/* set address first */
	mii_mgr_cl45_set_address(port_num, dev_addr, reg_addr);
	/* udelay(10); */

	enable_mdio(1);
	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			break;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			enable_mdio(0);
			pr_err("\n MDIO Read operation is ongoing !!\n");
			return rc;
		}
	}

	data =
	    (dev_addr << 25) | (port_num << 20) | (0x01 << 18) | (0x00 << 16) |
	    write_data;
	sys_reg_write(MDIO_PHY_CONTROL_0, data);
	sys_reg_write(MDIO_PHY_CONTROL_0, (data | (1 << 31)));

	t_start = jiffies;

	while (1) {
		if (!(sys_reg_read(MDIO_PHY_CONTROL_0) & (0x1 << 31))) {
			enable_mdio(0);
			return 1;
		} else if (time_after(jiffies, t_start + 5 * HZ)) {
			enable_mdio(0);
			pr_err("\n MDIO Write operation Time Out\n");
			return 0;
		}
	}
}
EXPORT_SYMBOL(mii_mgr_write_cl45);

void smi_init(void)
{
	//spin_lock_init(&g_mdio_lock);
	//mutex_init(mii_lock);
	return;
}
//UBNT_Andrew

static void rtlglue_drvMutexLock(void)
{
	//spin_lock_irqsave(&g_mdio_lock, flags);
	mutex_lock(&mii_lock);
	
    /* It is empty currently. Implement this function if Lock/Unlock function is needed */
    return;
}

static void rtlglue_drvMutexUnlock(void)
{
	//spin_unlock_irqrestore(&g_mdio_lock, flags);
	mutex_unlock(&mii_lock);
	
    /* It is empty currently. Implement this function if Lock/Unlock function is needed */
    return;
}



#if defined(MDC_MDIO_OPERATION) || defined(SPI_OPERATION)
    /* No local function in MDC/MDIO & SPI mode */
#else
static void _smi_start(void)
{

    /* change GPIO pin to Output only */
    GPIO_DIRECTION_SET(smi_SCK, GPIO_DIR_OUT);
    GPIO_DIRECTION_SET(smi_SDA, GPIO_DIR_OUT);

    /* Initial state: SCK: 0, SDA: 1 */
    GPIO_DATA_SET(smi_SCK, 0);
    GPIO_DATA_SET(smi_SDA, 1);
    CLK_DURATION(DELAY);

    /* CLK 1: 0 -> 1, 1 -> 0 */
    GPIO_DATA_SET(smi_SCK, 1);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SCK, 0);
    CLK_DURATION(DELAY);

    /* CLK 2: */
    GPIO_DATA_SET(smi_SCK, 1);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SDA, 0);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SCK, 0);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SDA, 1);

}



static void _smi_writeBit(rtk_uint16 signal, rtk_uint32 bitLen)
{
    for( ; bitLen > 0; bitLen--)
    {
        CLK_DURATION(DELAY);

        /* prepare data */
        if ( signal & (1<<(bitLen-1)) )
        {
            GPIO_DATA_SET(smi_SDA, 1);
        }
        else
        {
            GPIO_DATA_SET(smi_SDA, 0);
        }
        CLK_DURATION(DELAY);

        /* clocking */
        GPIO_DATA_SET(smi_SCK, 1);
        CLK_DURATION(DELAY);
        GPIO_DATA_SET(smi_SCK, 0);
    }
}



static void _smi_readBit(rtk_uint32 bitLen, rtk_uint32 *rData)
{
    rtk_uint32 u = 0;

    /* change GPIO pin to Input only */
    GPIO_DIRECTION_SET(smi_SDA, GPIO_DIR_IN);

    for (*rData = 0; bitLen > 0; bitLen--)
    {
        CLK_DURATION(DELAY);

        /* clocking */
        GPIO_DATA_SET(smi_SCK, 1);
        CLK_DURATION(DELAY);
        GPIO_DATA_GET(smi_SDA, &u);
        GPIO_DATA_SET(smi_SCK, 0);

        *rData |= (u << (bitLen - 1));
    }

    /* change GPIO pin to Output only */
    GPIO_DIRECTION_SET(smi_SDA, GPIO_DIR_OUT);
}



static void _smi_stop(void)
{

    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SDA, 0);
    GPIO_DATA_SET(smi_SCK, 1);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SDA, 1);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SCK, 1);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SCK, 0);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SCK, 1);

    /* add a click */
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SCK, 0);
    CLK_DURATION(DELAY);
    GPIO_DATA_SET(smi_SCK, 1);


    /* change GPIO pin to Input only */
    GPIO_DIRECTION_SET(smi_SDA, GPIO_DIR_IN);
    GPIO_DIRECTION_SET(smi_SCK, GPIO_DIR_IN);
}

#endif /* End of #if defined(MDC_MDIO_OPERATION) || defined(SPI_OPERATION) */

rtk_int32 smi_read(rtk_uint32 mAddrs, rtk_uint32 *rData)
{
#if (!defined(MDC_MDIO_OPERATION) && !defined(SPI_OPERATION))
    rtk_uint32 rawData=0, ACK;
    rtk_uint8  con;
    rtk_uint32 ret = RT_ERR_OK;
#endif

    if(mAddrs > 0xFFFF)
        return RT_ERR_INPUT;

    if(rData == NULL)
        return RT_ERR_NULL_POINTER;

#if defined(MDC_MDIO_OPERATION)

    /* Lock */
    rtlglue_drvMutexLock();

    /* Write address control code to register 31 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_PHY_ID, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

    /* Write address to register 23 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_PHY_ID, MDC_MDIO_ADDRESS_REG, mAddrs);

    /* Write read control code to register 21 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_PHY_ID, MDC_MDIO_CTRL1_REG, MDC_MDIO_READ_OP);

    /* Read data from register 25 */
    MDC_MDIO_READ(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_PHY_ID, MDC_MDIO_DATA_READ_REG, rData);

    /* Unlock */
    rtlglue_drvMutexUnlock();

    return RT_ERR_OK;

#elif defined(SPI_OPERATION)

    /* Lock */
    rtlglue_drvMutexLock();

    /* Write 8 bits READ OP_CODE */
    SPI_WRITE(SPI_READ_OP, SPI_READ_OP_LEN);

    /* Write 16 bits register address */
    SPI_WRITE(mAddrs, SPI_REG_LEN);

    /* Read 16 bits data */
    SPI_READ(rData, SPI_DATA_LEN);

    /* Unlock */
    rtlglue_drvMutexUnlock();

    return RT_ERR_OK;

#else

    /*Disable CPU interrupt to ensure that the SMI operation is atomic.
      The API is based on RTL865X, rewrite the API if porting to other platform.*/
    rtlglue_drvMutexLock();

    _smi_start();                                /* Start SMI */

    _smi_writeBit(0x0b, 4);                     /* CTRL code: 4'b1011 for RTL8370 */

    _smi_writeBit(0x4, 3);                        /* CTRL code: 3'b100 */

    _smi_writeBit(0x1, 1);                        /* 1: issue READ command */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for issuing READ command*/
    } while ((ACK != 0) && (con < ack_timer));

    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs&0xff), 8);             /* Set reg_addr[7:0] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[7:0] */
    } while ((ACK != 0) && (con < ack_timer));

    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs>>8), 8);                 /* Set reg_addr[15:8] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK by RTL8369 */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_readBit(8, &rawData);                    /* Read DATA [7:0] */
    *rData = rawData&0xff;

    _smi_writeBit(0x00, 1);                        /* ACK by CPU */

    _smi_readBit(8, &rawData);                    /* Read DATA [15: 8] */

    _smi_writeBit(0x01, 1);                        /* ACK by CPU */
    *rData |= (rawData<<8);

    _smi_stop();

    rtlglue_drvMutexUnlock();/*enable CPU interrupt*/

    return ret;
#endif /* end of #if defined(MDC_MDIO_OPERATION) */
}



rtk_int32 smi_write(rtk_uint32 mAddrs, rtk_uint32 rData)
{
#if (!defined(MDC_MDIO_OPERATION) && !defined(SPI_OPERATION))
    rtk_int8 con;
    rtk_uint32 ACK;
    rtk_uint32 ret = RT_ERR_OK;
#endif

    if(mAddrs > 0xFFFF)
        return RT_ERR_INPUT;

    if(rData > 0xFFFF)
        return RT_ERR_INPUT;

#if defined(MDC_MDIO_OPERATION)

    /* Lock */
    rtlglue_drvMutexLock();

    /* Write address control code to register 31 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_PHY_ID, MDC_MDIO_CTRL0_REG, MDC_MDIO_ADDR_OP);

    /* Write address to register 23 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_PHY_ID, MDC_MDIO_ADDRESS_REG, mAddrs);

    /* Write data to register 24 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_PHY_ID, MDC_MDIO_DATA_WRITE_REG, rData);

    /* Write data control code to register 21 */
    MDC_MDIO_WRITE(MDC_MDIO_PREAMBLE_LEN, MDC_MDIO_PHY_ID, MDC_MDIO_CTRL1_REG, MDC_MDIO_WRITE_OP);

    /* Unlock */
    rtlglue_drvMutexUnlock();

    return RT_ERR_OK;

#elif defined(SPI_OPERATION)

    /* Lock */
    rtlglue_drvMutexLock();

    /* Write 8 bits WRITE OP_CODE */
    SPI_WRITE(SPI_WRITE_OP, SPI_WRITE_OP_LEN);

    /* Write 16 bits register address */
    SPI_WRITE(mAddrs, SPI_REG_LEN);

    /* Write 16 bits data */
    SPI_WRITE(rData, SPI_DATA_LEN);

    /* Unlock */
    rtlglue_drvMutexUnlock();

    return RT_ERR_OK;
#else

    /*Disable CPU interrupt to ensure that the SMI operation is atomic.
      The API is based on RTL865X, rewrite the API if porting to other platform.*/
    rtlglue_drvMutexLock();

    _smi_start();                                /* Start SMI */

    _smi_writeBit(0x0b, 4);                     /* CTRL code: 4'b1011 for RTL8370*/

    _smi_writeBit(0x4, 3);                        /* CTRL code: 3'b100 */

    _smi_writeBit(0x0, 1);                        /* 0: issue WRITE command */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for issuing WRITE command*/
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs&0xff), 8);             /* Set reg_addr[7:0] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[7:0] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit((mAddrs>>8), 8);                 /* Set reg_addr[15:8] */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for setting reg_addr[15:8] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit(rData&0xff, 8);                /* Write Data [7:0] out */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                    /* ACK for writting data [7:0] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_writeBit(rData>>8, 8);                    /* Write Data [15:8] out */

    con = 0;
    do {
        con++;
        _smi_readBit(1, &ACK);                        /* ACK for writting data [15:8] */
    } while ((ACK != 0) && (con < ack_timer));
    if (ACK != 0) ret = RT_ERR_FAILED;

    _smi_stop();

    rtlglue_drvMutexUnlock();/*enable CPU interrupt*/

    return ret;
#endif /* end of #if defined(MDC_MDIO_OPERATION) */
}

