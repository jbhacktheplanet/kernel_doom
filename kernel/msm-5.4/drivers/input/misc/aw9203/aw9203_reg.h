#ifndef _AW9203_REG_H_
#define _AW9203_REG_H_

/********************************************
 * Register List
 *******************************************/
#define     AW9203_REG_RSTR             0x00
#define     AW9203_REG_GCR              0x01
#define     AW9203_REG_SLPR             0x02
#define     AW9203_REG_INTER            0x03
#define     AW9203_REG_OSR1             0x04
#define     AW9203_REG_OSR2             0x05
#define     AW9203_REG_OSR3             0x06
#define     AW9203_REG_AKSR             0x07
#define     AW9203_REG_SLSR             0x08
#define     AW9203_REG_INITTMR          0x09
#define     AW9203_REG_THR0             0x0a
#define     AW9203_REG_THR1             0x0b
#define     AW9203_REG_THR2             0x0c
#define     AW9203_REG_THR3             0x0d
#define     AW9203_REG_THR4             0x0e
#define     AW9203_REG_THR5             0x0f
#define     AW9203_REG_NOISTHR          0x10
#define     AW9203_REG_SCFG1            0x11
#define     AW9203_REG_SCFG2            0x12
#define     AW9203_REG_OFR1             0x13
#define     AW9203_REG_OFR2             0x14
#define     AW9203_REG_OFR3             0x15
#define     AW9203_REG_DOFCF0           0x16
#define     AW9203_REG_DOFCF1           0x17
#define     AW9203_REG_IDLECR           0x18
#define     AW9203_REG_MPTR             0x19
#define     AW9203_REG_DISMAX           0x1a
#define     AW9203_REG_SETCNT           0x1b
#define     AW9203_REG_BLCTH            0x1c
#define     AW9203_REG_BLDTH            0x1d
#define     AW9203_REG_MCR              0x1e
#define     AW9203_REG_ANAR             0x1f
#define     AW9203_REG_GDCFGR           0x20
#define     AW9203_REG_GDTR             0x21
#define     AW9203_REG_TDTR             0x22
#define     AW9203_REG_GESTR1           0x23
#define     AW9203_REG_GESTR2           0x24
#define     AW9203_REG_GESTR3           0x25
#define     AW9203_REG_GESTR4           0x26
#define     AW9203_REG_TAPR1            0x27
#define     AW9203_REG_GESTR5           0x28
#define     AW9203_REG_GESTR6           0x29
#define     AW9203_REG_GESTR7           0x2a
#define     AW9203_REG_GESTR8           0x2b
#define     AW9203_REG_TAPR2            0x2c
#define     AW9203_REG_GIER             0x2d
#define     AW9203_REG_GISR             0x2e
#define     AW9203_REG_GTIMR            0x2f
#define     AW9203_REG_RAWST            0x30
#define     AW9203_REG_AKSST            0x31
#define     AW9203_REG_ISR              0x32
#define     AW9203_REG_COR              0x33
#define     AW9203_REG_FGPRSR           0x34
#define     AW9203_REG_MOVCNTR          0x35
#define     AW9203_REG_KDATA0           0x36
#define     AW9203_REG_KDATA1           0x37
#define     AW9203_REG_KDATA2           0x38
#define     AW9203_REG_KDATA3           0x39
#define     AW9203_REG_KDATA4           0x3a
#define     AW9203_REG_KDATA5           0x3b
#define     AW9203_REG_DUM0             0x3c
#define     AW9203_REG_DUM1             0x3d
#define     AW9203_REG_SA_COX           0x3f

/********************************************
 * Register Access
 *******************************************/
#define REG_NONE_ACCESS   0
#define REG_RD_ACCESS     (1 << 0)
#define REG_WR_ACCESS     (1 << 1)

const unsigned char aw9203_reg_access[AW9203_REG_MAX] = {
	[AW9203_REG_RSTR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_SLPR] = REG_RD_ACCESS,
	[AW9203_REG_INTER] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_OSR1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_OSR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_OSR3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_AKSR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_SLSR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_INITTMR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_THR0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_THR1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_THR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_THR3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_THR4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_THR5] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_NOISTHR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_SCFG1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_SCFG2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_OFR1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_OFR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_OFR3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_DOFCF0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_DOFCF1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_IDLECR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_MPTR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_DISMAX] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_SETCNT] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_BLCTH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_BLDTH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_MCR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_ANAR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GDCFGR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GDTR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_TDTR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GESTR1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GESTR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GESTR3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GESTR4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_TAPR1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GESTR5] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GESTR6] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GESTR7] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GESTR8] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_TAPR2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GIER] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GISR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_GTIMR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_RAWST] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_AKSST] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_ISR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_COR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_FGPRSR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_MOVCNTR] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_KDATA0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_KDATA1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_KDATA2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_KDATA3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_KDATA4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_KDATA5] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_DUM0] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_DUM1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW9203_REG_SA_COX] = REG_RD_ACCESS | REG_WR_ACCESS,
};

/******************************************************
 * Register Detail
 *****************************************************/
#endif
