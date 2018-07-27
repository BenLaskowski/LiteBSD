/*
 * DDR pseudo-disk driver.
 *
 * Copyright (C) 2014 Ben Laskowski, <ben.laskowski@gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software
 * and its documentation for any purpose and without fee is hereby
 * granted, provided that the above copyright notice appear in all
 * copies and that both that the copyright notice and this
 * permission notice and warranty disclaimer appear in supporting
 * documentation, and that the name of the author not be used in
 * advertising or publicity pertaining to distribution of the
 * software without specific, written prior permission.
 *
 * The author disclaim all warranties with regard to this
 * software, including all implied warranties of merchantability
 * and fitness.  In no event shall the author be liable for any
 * special, indirect or consequential damages or any damages
 * whatsoever resulting from loss of use, data or profits, whether
 * in an action of contract, negligence or other tortious action,
 * arising out of or in connection with the use or performance of
 * this software.
 */

#include <ddr.h>
#if NDDR > 0

#include "ddr2_registers.h"

#define DDR_SIZE_KB 32768

#define COL_HI_RSHFT            0
#define COL_HI_MASK             0
#define COL_LO_MASK             ((1 << 9) - 1)

#define BA_RSHFT                9
#define BANK_ADDR_MASK          ((1 << 2) - 1)

#define ROW_ADDR_RSHIFT         (BA_RSHFT + 2)
#define ROW_ADDR_MASK           ((1 << 13) - 1)

#define CS_ADDR_RSHIFT          0
#define CS_ADDR_MASK            0

#define CTRL_CLK_PERIOD         (2500 * 2)

#define round_up(x,y) (((x) + (y) - 1) / (y))
#define sys_mem_ddr_max(a,b) (((a)>(b))?(a):(b))
#define sys_mem_ddr_hc_clk_dly(dly) (sys_mem_ddr_max((round_up((dly),2500)),2)-2)

#define DRV_DDR_IDLE_NOP                0x00FFFFFF
#define DRV_DDR_PRECH_ALL_CMD           0x00FFF401
#define DRV_DDR_REF_CMD                 0x00FFF801
#define DRV_DDR_LOAD_MODE_CMD           0x00FFF001
#define DRV_DDR_CKE_LOW                 0x00FFEFFE

#include <sys/param.h>
#include <sys/buf.h>
#include <sys/stat.h>
#include <sys/syslog.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/disklabel.h>
#include <sys/systm.h>
#include <sys/dkstat.h>

#include <mips/dev/device.h>
#include <mips/dev/spi.h>
#include <machine/pic32mz.h>

#include <machine/pic32_gpio.h>

// memcpy() is implemented in locore.s, but we need a prototype for it here
void* memcpy(void *dest, const void *src, size_t count);

// btod() was defined in sys/pic32/machparam.h in RetroBSD.  duplicate that definition here.
#define btod(x)         (((x) + DEV_BSIZE - 1) >> DEV_BSHIFT)

// DIOCGETMEDIASIZE was defined in sys/include/disk.h in RetroBSD.  duplicate that def here.
#define DIOCGETMEDIASIZE _IOR('d', 1, int)

// The swap space!
static unsigned char swap_memory[DDR_SIZE_KB * 1024] __attribute__ ((section(".ddr")));

int
ddropen(dev_t dev, int flags, int mode)
{
    printf("ddr0:  ddropen() minor = %u\n", minor(dev));
    if (minor(dev) != 1)
        return ENODEV;
    return 0;
}

int
ddrclose(dev_t dev, int flags, int mode)
{
    printf("ddr0:  ddrclose()\n");
    return 0;
}

/*
 * Control an LED to show DDR activity
 */
static inline void
ddrled(int val)
{
#ifdef DDR_LED_PORT
#ifndef DDR_LED_INVERT
    if (val)
        LAT_SET(DDR_LED_PORT) = 1 << DDR_LED_PIN;
    else
        LAT_CLR(DDR_LED_PORT) = 1 << DDR_LED_PIN;
#else
    if (val)
        LAT_CLR(DDR_LED_PORT) = 1 << DDR_LED_PIN;
    else
        LAT_SET(DDR_LED_PORT) = 1 << DDR_LED_PIN;
#endif
#endif
}

daddr_t ddrsize(dev_t dev)
{
    printf("ddr0:  ddrsize()\n");
    return DDR_SIZE_KB;
}

/*
 * Read/write routine for a buffer.  Finds the proper unit, range checks
 * arguments, and schedules the transfer.  Does not wait for the transfer
 * to complete.  Multi-page transfers are supported.  All I/O requests must
 * be a multiple of a sector in length.
 */
void
ddrstrategy(bp)
    struct buf *bp;
{
    int offset = bp->b_blkno;
    long nblk = btod(bp->b_bcount);
    int s;
/*
    if (bp->b_flags & B_READ)
        printf("ddr0:  ddrstrategy() reading %u bytes from 0x%08x\n",bp->b_bcount, &swap_memory[offset << DEV_BSHIFT]);
    else
        printf("ddr0:  ddrstrategy() writing %u bytes to 0x%08x\n",bp->b_bcount, &swap_memory[offset << DEV_BSHIFT]);
  */  
    // Determine size of the transfer and make sure it is within boundaries of the swap space
    if (bp->b_blkno + nblk > DDR_SIZE_KB)
    {
        // if exactly at end of partition, return EOF
        if (bp->b_blkno == DDR_SIZE_KB)
        {
            bp->b_resid = bp->b_bcount;
            biodone(bp);
            return;
        }

        // or, truncate if part of it fits
        nblk = DDR_SIZE_KB - bp->b_blkno;
        if (nblk <= 0)
        {
            bp->b_error = EINVAL;
            bp->b_flags |= B_ERROR;
            biodone(bp);
            return;
        }

        bp->b_bcount = nblk << DEV_BSHIFT;
    }

    ddrled(1);

    s = splbio();

    if (bp->b_flags & B_READ)
    {
        memcpy(bp->b_un.b_addr, &swap_memory[offset << DEV_BSHIFT], bp->b_bcount);
    }
    else
    {
        memcpy(&swap_memory[offset << DEV_BSHIFT], bp->b_un.b_addr, bp->b_bcount);
    }

    biodone(bp);

    ddrled(0);

    splx(s);
}

int
ddrioctl(dev, cmd, data, flag, p)
    dev_t dev;
    u_long cmd;
    caddr_t data;
    int flag;
    struct proc *p;
{
    printf("ddr0:  ddrioctl()\n");
    int error = 0;
    switch(cmd)
    {
        case DIOCGETMEDIASIZE:
            *(int*) data = DDR_SIZE_KB;
            break;
        default:
            error = EINVAL;
            break;
    }
    return error;
}

int
ddrdump(dev_t dev)
{
    printf("ddr0:  ddrdump()\n");
    return ENXIO;
}

int
ddrread(dev, uio)
    dev_t dev;
    struct uio *uio;
{
    return physio(ddrstrategy, 0, dev, B_READ, minphys, uio);
}

int
ddrwrite(dev, uio)
    dev_t dev;
    struct uio *uio;
{
    return physio(ddrstrategy, 0, dev, B_WRITE, minphys, uio);
}

static void
ddr_clock_init()
{
    SYSKEY = 0;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    
    // enable PLL voltage regulator with external reference and wait for it to be ready
    CFGMPLL &= ~(1 << 22);
    while (!(CFGMPLL & (1 << 23)));
    CFGMPLL &= ~(3 << 6);

    // set up PLL - input div 8, mult 50, div 2, div 1
    CFGMPLL &= ~(0x3F00FF3F);
    CFGMPLL |= 0x0A003203;

    // enable PLL and wait for ready
    CFGMPLL &= ~(1 << 30);
    while (!(CFGMPLL & (1 << 31)));  
}

static void
ddr_pmd_init()
{
    // power up the DDR
    CFGCON &= ~(1 << 12);
    PMD7 &= ~(1 << 28);
    CFGCON |= (1 << 12);
    SYSKEY = 0x33333333;
}

static void
ddr_phy_init()
{
    DDRPHYPADCON |= 3;
    DDRPHYPADCON &= ~(1 << 2); 
    DDRPHYPADCON &= ~(1 << 3); 
    DDRPHYPADCON |= 3 << 6;
    DDRPHYPADCON |= 2 << 4;
    DDRPHYPADCON |= 14 << 20;
    DDRPHYPADCON |= 14 << 16;
    DDRPHYPADCON &= ~(1 << 8);
    DDRPHYPADCON |= 1 << 9;
    DDRPHYPADCON |= 1 << 28;
    DDRPHYPADCON |= 2 << 29;
    DDRPHYPADCON |= 1 << 14;
    DDRPHYPADCON |= 1 << 13;
    DDRPHYDLLR |= 16 << 8;
    DDRPHYDLLR &= ~(1 << 26);
    DDRPHYDLLR |= 3 << 28;
    DDRSCLCFG0 |= 1;
    DDRSCLCFG0 |= 2;
    DDRSCLCFG0 = (DDRSCLCFG0 & ~(0xF0)) | (5 << 4);
    DDRSCLCFG1 = (DDRSCLCFG1 & ~(0xF00)) | (4 << 8);
    DDRSCLCFG0 |= 1 << 24;
    DDRSCLCFG1 &= ~(1 << 12); 
    DDRSCLLAT = 0x43; 
}

static void
ddr_arbitration_init(int target, int min_limit, int req_period, int min_cmd)
{
    DDRTSEL = 5 * target;
    DDRMINLIM = min_limit;
    DDRTSEL = 8 * target;
    DDRRQPER = req_period;
    DDRTSEL = 8 * target;
    DDRMINCMD = min_cmd;
}

static void
ddr_init()
{
    // arbitration init
    ddr_arbitration_init(0, 0x1F, 0xFF, 0x04);
    ddr_arbitration_init(1, 0x1F, 0xFF, 0x10);
    ddr_arbitration_init(2, 0x1F, 0xFF, 0x10);
    ddr_arbitration_init(3, 0x04, 0xFF, 0x04);
    ddr_arbitration_init(4, 0x04, 0xFF, 0x04);
    
    // addressing - rows
    DDRMEMCFG0 |= ROW_ADDR_RSHIFT;
    DDRMEMCFG1 = ROW_ADDR_MASK;
    
    // addressing - columns
    DDRMEMCFG0 |= COL_HI_RSHFT << 24;
    DDRMEMCFG3 = COL_LO_MASK;
    DDRMEMCFG2 = COL_HI_MASK;
    
    // addressing - banks
    DDRMEMCFG0 |= BA_RSHFT << 8;
    DDRMEMCFG4 |= BANK_ADDR_MASK;
    
    // addressing - chip selects
    DDRMEMCFG0 |= CS_ADDR_RSHIFT << 16;
    DDRMEMCFG4 |= CS_ADDR_MASK << 6;
    
    // refresh
    DDRREFCFG |= ((7800000 + CTRL_CLK_PERIOD - 1) / CTRL_CLK_PERIOD - 2);
    DDRREFCFG |= ((127500 + CTRL_CLK_PERIOD - 1) / CTRL_CLK_PERIOD - 2) << 16;
    DDRREFCFG |= 7 << 24;
    DDRPWRCFG &= ~(1 << 3);

    // power
    DDRPWRCFG &= ~(1 << 2);
    DDRMEMCFG0 &= ~(1 << 30);
    DDRPWRCFG &= ~(1 << 22);

    // timing
    DDRDLYCFG0 |= (2 + 2) << 24;
    DDRDLYCFG0 |= (5 - 4 + 3) << 28;
    uint32_t w2rdly, w2rcsdly;
    w2rdly = round_up(7500, CTRL_CLK_PERIOD) + 2 + 4;
    w2rcsdly = ((w2rdly - 1) > 3) ? (w2rdly - 1) : 3;
    DDRDLYCFG0 |= (w2rdly & 0x0F);
    DDRDLYCFG1 |= (((w2rdly & 0x10) != 0) ? 1 : 0) << 27;
    DDRDLYCFG0 |= (w2rcsdly & 0x0F) << 4;
    DDRDLYCFG1 |= (((w2rcsdly & 0x10) != 0) ? 1 : 0) << 28;
    DDRDLYCFG0 |= (2 - 1) << 8;
    DDRDLYCFG0 |= 2 << 12;
    DDRDLYCFG0 |= (2 - 1) << 16;
    DDRDLYCFG0 |= (2 - 1) << 20;
    DDRPWRCFG |= 17 << 24;
    DDRDLYCFG1 |= 3 - 1;
    DDRDLYCFG1 |= ((round_up(200, 2) - 2) & 0xFF) << 8;
    DDRDLYCFG1 |= (((round_up(200, 2) & 0x100u) != 0) ? 1 : 0) << 30;
    DDRPWRCFG |= 8 << 4;
    DDRDLYCFG1 |= (3 - 1) << 8;
    DDRDLYCFG1 |= ((2 > 3 ? 2 : 3) - 1) << 12;
    DDRDLYCFG2 |= round_up(12500, CTRL_CLK_PERIOD);
    DDRDLYCFG2 |= (round_up(7500, CTRL_CLK_PERIOD) + 2 - 2) << 8;
    DDRDLYCFG2 |= ((round_up(15000, CTRL_CLK_PERIOD) + 4 + 2) & 0x0F) << 12;
    DDRDLYCFG1 |= ((round_up(15000, CTRL_CLK_PERIOD) + 4 + 2) & 0x10) << 26;
    DDRDLYCFG2 |= (round_up(12500, CTRL_CLK_PERIOD) - 1) << 24;
    DDRDLYCFG3 |= round_up(45000, CTRL_CLK_PERIOD) - 1;
    DDRDLYCFG3 |= (round_up(57500, CTRL_CLK_PERIOD) - 1) << 8;
    DDRDLYCFG2 |= (round_up(7500, CTRL_CLK_PERIOD) - 1) << 16;
    DDRDLYCFG2 |= (round_up(12500, CTRL_CLK_PERIOD) - 1) << 24;
    DDRDLYCFG2 |= (5 + 3) << 28;
    DDRXFERCFG |= 2;
    DDRXFERCFG |= 4 << 4;
    DDRDLYCFG1 |= ((((5 + 5) & 0x10u) != 0) ? 1 : 0) << 29;
    DDRXFERCFG |= 2 << 16;
    DDRDLYCFG3 |= (round_up(35000, CTRL_CLK_PERIOD) - 1) << 16;

    // on-die termination
    DDRODTCFG &= ~(0xFF);
    DDRODTENCFG &= ~(1 << 0);
    DDRODTCFG &= ~(0xFF);
    DDRODTENCFG &= ~(1 << 16);

    // controller settings
    DDRXFERCFG &= ~(1 << 31);
    DDRMEMWIDTH |= 1 << 3;
    DDRXFERCFG |= 3 << 24;
    DDRCMDISSUE |= 12;

    // dram initialization

    // bring CKE high after reset and wait 400 nsec
    DDRCMD10 = DRV_DDR_IDLE_NOP;
    DDRCMD20 = (0x00 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(400000) << 11));

    // issue precharge all command
    DDRCMD11 = DRV_DDR_PRECH_ALL_CMD;
    DDRCMD21 = (0x04 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(12500 + 2500) << 11));

    // initialize EMR2
    DDRCMD12 = DRV_DDR_LOAD_MODE_CMD;
    DDRCMD22 = (0x00 | (0x02 << 8) | (sys_mem_ddr_hc_clk_dly(2 * 2500) << 11));

    // initialize EMR3
    DDRCMD13 = DRV_DDR_LOAD_MODE_CMD;
    DDRCMD23 = (0x00 | (0x03 << 8) | (sys_mem_ddr_hc_clk_dly(2 * 2500) << 11));

    // RDQS disable, DQSB enable, OCD exit, 150 ohm termination, AL=0, DLL enable
    DDRCMD14 = (DRV_DDR_LOAD_MODE_CMD | (0x40 << 24));
    DDRCMD24 = (0x00 | (0x01 << 8) | (sys_mem_ddr_hc_clk_dly(2 * 2500) << 11));

    uint32_t tmp, ma_field, ba_field;
    tmp = ((round_up(15000, 2500) -1 ) << 1) | 1;
    ma_field = tmp & 0xFF;
    ba_field = (tmp >> 8) & 0x03;

    // PD fast exit, WR REC = tWR in clocks -1, DLL reset, CAS = RL, burst = 4
    DDRCMD15 = (DRV_DDR_LOAD_MODE_CMD | (((5 << 4) | 2) << 24));
    DDRCMD25 = (ma_field | (ba_field << 8) | (sys_mem_ddr_hc_clk_dly(2 * 2500) << 11));

    // issue precharge all command
    DDRCMD16 = DRV_DDR_PRECH_ALL_CMD;
    DDRCMD26 = (0x04 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(12500 + 2500) << 11));

    // issue refresh command
    DDRCMD17 = DRV_DDR_REF_CMD;
    DDRCMD27 = (0x00 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(127500) << 11));

    // issue refresh command
    DDRCMD18 = DRV_DDR_REF_CMD;
    DDRCMD28 = (0x00 | (0x00 << 8) | (sys_mem_ddr_hc_clk_dly(127500) << 11));
	
    tmp = ((round_up(15000, 2500) -1 ) << 1);
    ma_field = tmp & 0xFF;
    ba_field = (tmp >> 8) & 0x03;

    // Mode register programming as before without DLL reset
    DDRCMD19 = (DRV_DDR_LOAD_MODE_CMD | (((5 << 4) | 3) << 24));
    DDRCMD29 = (ma_field | (ba_field << 8) | (sys_mem_ddr_hc_clk_dly(2 * 2500) << 11));

    // extended mode register same as before with OCD default
    DDRCMD110 = (DRV_DDR_LOAD_MODE_CMD | (0xC0 << 24));
    DDRCMD210 = (0x03 | (0x01 << 8) | (sys_mem_ddr_hc_clk_dly(2 * 2500) << 11));

    // extended mode register same as before with OCD exit
    DDRCMD111 = (DRV_DDR_LOAD_MODE_CMD | (0x40 << 24));
    DDRCMD211 = (0x00 | (0x01 << 8) | (sys_mem_ddr_hc_clk_dly(140 * 2500) << 11));

    // Set number of host commands
    DDRCMDISSUE = (DDRCMDISSUE & ~(0x0F)) | 0x1B;
	
    DDRCMDISSUE |= 1 << 4;
    DDRMEMCON |= 1;
    while (DDRCMDISSUE & (1 << 4));
    DDRMEMCON |= 2;
}

static void
ddr_phy_calib()
{
    DDRSCLSTART |= 1 << 26;
    DDRSCLSTART |= 1 << 28;
    while ((DDRSCLSTART & 3) != 3);
}

// Function that can initialize DDR elsewhere
// If this is called, we don't re-initialize later
void ddr_initialize()
{
    static int initialized = 0;
    
    if (initialized != 0)
    {
        printf("ddr0:  already initialized\n");
        return;
    }
    
    printf("ddr0:  initializing ... ");
    ddr_clock_init();
    ddr_pmd_init();
    ddr_phy_init();
    ddr_init();
    ddr_phy_calib();
    printf("done\n");
    
    initialized = 1;    
    return;
}

/*
 * Test to see if device is present.
 * Return true if found and initialized ok.
 */
static int
ddrprobe(config)
    struct conf_device *config;
{
    // only one unit is supported
    if (config->dev_unit != 0)
        return 0;

    // initialize DDR2 controller
    int x = splhigh();
    ddr_initialize();
    splx(x);

    printf("ddr0:  %u kbytes swap space\n", DDR_SIZE_KB);

    /* Configure LED pin as output. */
#ifdef DDR_LED_PORT
    ANSEL_CLR(DDR_LED_PORT) = 1 << DDR_LED_PIN;
    TRIS_CLR(DDR_LED_PORT) = 1 << DDR_LED_PIN;
#endif
    return 1;
}

struct driver ddrdriver = {
    "ddr", ddrprobe,
};

#endif  // NDDR > 0
