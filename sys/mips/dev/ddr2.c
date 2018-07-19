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
#define DDR_SIZE_KB 32768

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
    if (minor(dev) > 0)
        return ENODEV;
    return 0;
}

int
ddrclose(dev_t dev, int flags, int mode)
{
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
}

static void
ddr_pmd_init()
{
}

static void
ddr_phy_init()
{
}

static void
ddr_init()
{
}

static void
ddr_phy_calib()
{
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
    ddr_clock_init();
    ddr_pmd_init();
    ddr_phy_init();
    ddr_init();
    ddr_phy_calib();
    splx(x);

    printf("ddr0:  %u kbytes swap space\n", DDR_SIZE_KB);

    /* Configure LED pin as output. */
#ifdef DDR2_LED_PORT
    ANSEL_CLR(DDR_LED_PORT) = 1 << DDR_LED_PIN;
    TRIS_CLR(DDR_LED_PORT) = 1 << DDR_LED_PIN;
#endif
    return 1;
}

struct driver ddrdriver = {
    "ddr", ddrprobe,
};

#endif  // NDDR > 0
