/*
 * SD flash card disk driver using Secure Digital Host Controller
 *
 * Copyright (C) 2018 Ben Laskowski, <ben.laskowski@gmail.com>
 * Based heavily on the work of Serge Vakulenko
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
#include "sdhc.h"
#if NSDHC > 0

#include "sdhc_registers.h"
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
#include <machine/pic32mz.h>

#include <machine/pic32_gpio.h>

#define sdhcunit(dev)     ((minor(dev) & 8) >> 3)
#define sdhcpart(dev)     ((minor(dev) & 7))
#define RAWPART         0           /* whole disk */

#define NPARTITIONS     4
#define SECTSIZE        512
#define MBR_MAGIC       0xaa55

#ifndef SDHC_KHZ
#define SDHC_KHZ          12500       /* speed 12.5 MHz */
#endif
#ifndef SDHC_FAST_KHZ
#define SDHC_FAST_KHZ     25000       /* up to 25 Mhz is allowed by the spec */
#endif
#ifndef SDHC_FASTEST_KHZ
#define SDHC_FASTEST_KHZ  50000      /* max speed for pic32mz SDHC is 50 MHz */
#endif

#if DEV_BSIZE != 512
#error Only 512-byte block size supported.
#endif

/*
 * The structure of a disk drive.
 */
struct disk {
    /*
     * Partition table.
     */
    struct diskpart part [NPARTITIONS+1];

    /*
     * Card type.
     */
    int     card_type;
#define TYPE_UNKNOWN    0
#define TYPE_SD_LEGACY  1
#define TYPE_SD_II      2
#define TYPE_SDHC       3

    int     unit;           /* physical unit number */
    int     open;           /* open/closed refcnt */
    int     wlabel;         /* label writable? */
    int     dkindex;        /* disk index for statistics */
    u_int   copenpart;      /* character units open on this drive */
    u_int   bopenpart;      /* block units open on this drive */
    u_int   openpart;       /* all units open on this drive */
    u_int   ocr;            /* operation condition register */
    u_int   rca;            /* relative card address */
    u_char  csd[16];        /* card-specific data */
#define TRANS_SPEED_25MHZ   0x32
#define TRANS_SPEED_50MHZ   0x5a
#define TRANS_SPEED_100MHZ  0x0b
#define TRANS_SPEED_200MHZ  0x2b

    u_short group[6];       /* function group bitmasks */
    int     ma;             /* power consumption */
};

static struct disk sdhcdrives[NSDHC];       /* Table of units */

#define TIMO_WAIT_WDONE 50
#define TIMO_WAIT_WIDLE 50
#define TIMO_WAIT_CMD   50
#define TIMO_WAIT_WDATA 50
#define TIMO_READ       50
#define TIMO_SEND_OP    50
#define TIMO_CMD        50
#define TIMO_SEND_CSD   50
#define TIMO_WAIT_WSTOP 50

int sd_timo_cmd;                /* Max timeouts, for sysctl */
int sd_timo_send_op;
int sd_timo_send_csd;
int sd_timo_read;
int sd_timo_wait_cmd;
int sd_timo_wait_wdata;
int sd_timo_wait_wdone;
int sd_timo_wait_wstop;
int sd_timo_wait_widle;

/*
 * Definitions for MMC/SDC commands.
 */
#define CMD_GO_IDLE             0       /* CMD0 */
#define CMD_SEND_OP_MMC         1       /* CMD1 (MMC) */
#define CMD_ALL_SEND_CID        2
#define CMD_SEND_REL_ADDR       3
#define CMD_SWITCH_FUNC         6
#define CMD_SET_BUS_WIDTH       6 | 64  /* ACMD6, a duplicate, so we OR it with 64 */
#define CMD_SELECT_CARD         7
#define CMD_SEND_IF_COND        8
#define CMD_SEND_CSD            9
#define CMD_SEND_CID            10
#define CMD_STOP                12
#define CMD_SEND_STATUS         13      /* CMD13 */
#define CMD_SET_BLEN            16
#define CMD_READ_SINGLE         17
#define CMD_READ_MULTIPLE       18
#define CMD_SET_BCOUNT          23      /* (MMC) */
#define CMD_SET_WBECNT          23      /* ACMD23 (SDC) */
#define CMD_WRITE_SINGLE        24
#define CMD_WRITE_MULTIPLE      25
#define CMD_SEND_OP_SDC         41      /* ACMD41 (SDC) */
#define CMD_APP                 55      /* CMD55 */
#define CMD_READ_OCR            58

#define DATA_START_BLOCK        0xFE    /* start data for single block */
#define STOP_TRAN_TOKEN         0xFD    /* stop token for write multiple */
#define WRITE_MULTIPLE_TOKEN    0xFC    /* start data for write multiple */ 

/*
 * Wait while busy, up to 300 msec.
 */
static void sdhc_wait_ready(int limit, int *maxcount)
{
    int i, j;

    for (i=0; i<limit; i++)
    {
        for (j=0; j<100; j++)   // 100 is a fudge factor of sorts
        {
            if (0 == (SDHCSTAT1 & 3))
            {
                if (*maxcount < i)
                    *maxcount = i;
                return;
            }
        }
    }
    *maxcount = limit;
    printf("sdhc:  wait_ready(%d) failed\n", limit);
}

// send a command and address to the card.  it is up to the calling function to get and parse the response.
// returns 0 on success, 1 on failure.  If the card doesn't time out in the early stages of the command,
// it's up to the caller to parse the response registers and/or interrupt flags.
static int card_cmd(unsigned int unit, unsigned int cmd, unsigned int arg, unsigned int n_sectors)
{
    // Wait for not busy
    if (cmd != CMD_GO_IDLE)
    {
        sdhc_wait_ready(TIMO_WAIT_CMD, &sd_timo_wait_cmd);
        if (sd_timo_wait_cmd == TIMO_WAIT_CMD)
        {
            printf("sdhc:  card not ready\n");
            return 1;
        }
    }

    // set command argument
    SDHCARG = arg;

    // supply sector count
    // this only really matters for read/write multiple commands, but we do it here all the time anyway
    // 512 byte block size, or 64 bytes for CMD6, number of blocks in upper word
    if (cmd == CMD_SWITCH_FUNC) SDHCBLKCON = 64;
    else                        SDHCBLKCON = 512;
    SDHCBLKCON |= (n_sectors & 0xFFFF) << 16;

    // set up command
    // we can't write the command register until we know the entire bit pattern
    unsigned int shadow = (cmd & 63) << 24;

    // set abort bits for CMD12
    if (cmd == CMD_STOP)
        shadow |= 3 << 22;

    // data present for read/write single/multiple commands and CMD6
    if (cmd == CMD_SWITCH_FUNC ||
        cmd == CMD_READ_SINGLE || cmd == CMD_READ_MULTIPLE ||
        cmd == CMD_WRITE_SINGLE || cmd == CMD_WRITE_MULTIPLE)
        shadow |= 1 << 21;

    // don't check command index for CMD2, CMD9, CMD10, ACMD41
    if (cmd != CMD_ALL_SEND_CID && cmd != CMD_SEND_CSD && cmd != CMD_SEND_CID && cmd != CMD_SEND_OP_SDC)
        shadow |= 1 << 20;

    // don't check CRC for CMD9 or ACMD41
    if (/*cmd != CMD_SEND_CSD &&*/ cmd != CMD_SEND_OP_SDC)
        shadow |= 1 << 19;

    // response types
    // no response:  CMD0
    // 48-bit response without busy (R1, R3, R6, R7):  CMD3, ACMD6, CMD8, CMD13, CMD16, CMD17, CMD18, AMCD23, CMD24, CMD25, ACMD41, CMD55
    // 48-bit response with busy (R1b):  CMD7, CMD12
    // 136-bit response (R2):  CMD2, CMD9, CMD10
    if (cmd == CMD_GO_IDLE)
        shadow |= 0 << 16;  // no response
    else if (cmd == CMD_SELECT_CARD || cmd == CMD_STOP)
        shadow |= 3 << 16;  // R1b
    else if (cmd == CMD_ALL_SEND_CID || cmd == CMD_SEND_CSD || cmd == CMD_SEND_CID)
        shadow |= 1 << 16;  // R2
    else
        shadow |= 2 << 16;  // R1

    // multiple block select
    if ((cmd == CMD_READ_MULTIPLE || cmd == CMD_WRITE_MULTIPLE) && n_sectors > 1)
        shadow |= 1 << 5;

    // data transfer direction is write unless executing a read command
    if (cmd == CMD_READ_SINGLE || cmd == CMD_READ_MULTIPLE || cmd == CMD_SWITCH_FUNC)
        shadow |= 1 << 4;

    // use auto CMD12 for read and write multiple
    if (cmd == CMD_READ_MULTIPLE || cmd == CMD_WRITE_MULTIPLE)
        shadow |= 1 << 2;
    
    // use block count register for any commmand that reads or writes
    if (cmd == CMD_SWITCH_FUNC || cmd == CMD_READ_MULTIPLE || cmd == CMD_WRITE_MULTIPLE ||
        cmd == CMD_READ_SINGLE || cmd == CMD_WRITE_SINGLE)
        shadow |= 1 << 1;

    // clear interrupt flags by setting them (?!)
    // then enable all the interrupts we need to check
    unsigned mask = 0x03FF81FF; //0x03FF8033;
    SDHCINTSTAT |= mask;
    SDHCINTEN   |= mask; 


    // use PIO transfers for now, maybe try DMA later
    
    // send the command!
    SDHCMODE = shadow;

    //printf("sdhc:  SDHCMODE = %x, SDHCBLKCON = %x, SDHCARG = %x, SDHCINTSTAT = %x\n", 
    //    shadow, SDHCBLKCON, SDHCARG, SDHCINTSTAT);

    // wait for command complete or timeout
    while (!(SDHCINTSTAT & mask));      

    // debugging statements
    //if (SDHCINTSTAT & ((1 << 16) | (1 << 20)))
    //    printf("sdhc:  timeout, SDHCINTSTAT = %x\n", SDHCINTSTAT);
    //else
    //    printf("sdhc:  success\n");

    // timeout or other error
    if (SDHCINTSTAT & 0x03FF0000)
        return 1;
    
    // done!  let the caller parse the response registers
    //printf("sdhc:  cmd %u success, SDHCRESP0 = %x\n", cmd, SDHCRESP0);
    return 0;
}

/*
 * Control an LED to show SD activity
 */
static inline void
sdhc_led(int val)
{
#ifdef SD_LED_PORT
#ifndef SD_LED_INVERT
    if (val)
        LAT_SET(SD_LED_PORT) = 1 << SD_LED_PIN;
    else
        LAT_CLR(SD_LED_PORT) = 1 << SD_LED_PIN;
#else
    if (val)
        LAT_CLR(SD_LED_PORT) = 1 << SD_LED_PIN;
    else
        LAT_SET(SD_LED_PORT) = 1 << SD_LED_PIN;
#endif
#endif
}

// set the clock speed for the card
static void sdhc_set_speed(unsigned int speed)
{
    int base_clk = CPU_KHZ / 2;
    int divisor;    

    // we only have a few dividers to choose from, so pick the smallest
    // that gives an SD clock less than or equal to the target.
    for (divisor = 1; divisor < 256; divisor <<= 1)
        if (base_clk / divisor <= speed) break;

    printf("sdhc:  using clock divisor of %u to obtain %d kHz\n", divisor, speed);
    divisor = (divisor >> 1) & 0xFF;
    
    // disable SDHC clocks
    SDHCCON2 = 0;
    
    // set data timeout to maximum
    SDHCCON2 |= 0x0F << 16;
    

    // set divisor
    SDHCCON2 |= divisor << 8;

    // enable internal oscillator and wait for stability
    SDHCCON2 |= 1;
    while (!(SDHCCON2 & 2));

    // enable SD clock
    SDHCCON2 |= 4;
    
    printf("sdhc:  done waiting for clock\n");

    // done!
    return;
}    

/*
 * Initialize a card.
 * Return nonzero if successful.
 */
static int card_init(int unit)
{
    struct disk *u = &sdhcdrives[unit];
    int i, reply;
    int timeout = 4;

    /* Slow speed: 400 kHz */
    sdhc_set_speed(400);

    u->card_type = TYPE_UNKNOWN;

    sdhc_led(1);

    do {
        timeout--;
        reply = card_cmd(unit, CMD_GO_IDLE, 0, 0);

    } while ((reply != 0) && (timeout != 0));

    if (reply != 0)
    {
        // CMD_GO_IDLE must not return error
        sdhc_led(0);
        return 0;
    }

    /* Check SD version. */
    reply = card_cmd(unit, CMD_SEND_IF_COND, 0x1AA, 0);
    if (reply == 1)
        // CMD8 failure - Type I card
        u->card_type = TYPE_SD_LEGACY;
    else if ((SDHCRESP0 & 0xFF) == 0xAA)
        // CMD8 success and check pattern ok- Type II card
        u->card_type = TYPE_SD_II;
    else
    {
        printf("sdhc:  cannot detect card type, response=%x\n", SDHCRESP0);
        sdhc_led(0);
        return 0;
    }

    /* Send repeatedly SEND_OP until Idle terminates. */
    for (i=0; ; i++)
    {
        card_cmd(unit, CMD_APP, 0, 0);
        reply = card_cmd(unit, CMD_SEND_OP_SDC,
                         (u->card_type == TYPE_SD_II) ? 0x40FF8000 : 0xFF8000, 0);
        if (reply == 0 && (SDHCRESP0 & 1 << 31))
            break;
        if (i >= TIMO_SEND_OP)
        {
            /* Init timed out. */
            printf("card_init: SEND_OP timed out, reply = %d, SDHCRES0 = %d\n", reply, SDHCRESP0);
            sdhc_led(0);
            return 0;
        }
    }
    if (sd_timo_send_op < i)
        sd_timo_send_op = i;

    if (u->card_type == TYPE_SD_II)
    {
        u->ocr = SDHCRESP0;
        if((u->ocr & 0xC0000000) == 0xC0000000)
            u->card_type = TYPE_SDHC;
    }

    // use CMD_ALL_SEND_CID to bump card to init state
    reply = card_cmd(unit, CMD_ALL_SEND_CID, 0, 0);
    if (reply != 0)
    {
        printf("sdhc:  CMD_ALL_SEND_CID failed, INTSTAT = %x\n", SDHCINTSTAT);
        sdhc_led(0);
        return 0;
    }

    // use CMD_SEND_RELATIVE_ADDRESS to get card address and finish initialization
    reply = card_cmd(unit, CMD_SEND_REL_ADDR, 0, 0);
    if (reply != 0)
    {
        printf("sdhc:  CMD_SEND_RELATIVE_ADDRESS failed, INTSTAT = %x\n", SDHCINTSTAT);
        sdhc_led(0);
        return 0;
    }
    u->rca = SDHCRESP0 & 0xFFFF0000;
       
    return 1;
}

/*
 * Get the value of CSD register.
 */
static int card_read_csd(int unit)
{
    struct disk *u = &sdhcdrives[unit];
    int reply;

    sdhc_led(1);

    reply = card_cmd(unit, CMD_SEND_CSD, u->rca, 0);
/*    
    printf( "sdhc:  SEND_CSD:  RESP0 = %8x\n"
            "                  RESP1 = %8x\n"
            "                  RESP2 = %8x\n"
            "                  RESP3 = %8x\n"
            "                INTSTAT = %8x\n",
        SDHCRESP0, SDHCRESP1, SDHCRESP2, SDHCRESP3, SDHCINTSTAT);
*/
    if (reply != 0)
    {
        printf("sdhc:  CMD_SEND_CSD failed\n");
        sdhc_led(0);
        return 0;
    }

    /* Read data. */
    // ugly code, but it lets us keep Serge's CSD parsing code
    /* first attempt
    u->csd[0]  = SDHCRESP0 >> 24;
    u->csd[1]  = SDHCRESP0 >> 16;
    u->csd[2]  = SDHCRESP0 >> 8;
    u->csd[3]  = SDHCRESP0 & 0xFF;
    u->csd[4]  = SDHCRESP1 >> 24;
    u->csd[5]  = SDHCRESP1 >> 16;
    u->csd[6]  = SDHCRESP1 >> 8;
    u->csd[7]  = SDHCRESP1 & 0xFF;
    u->csd[8]  = SDHCRESP2 >> 24;
    u->csd[9]  = SDHCRESP2 >> 16;
    u->csd[10] = SDHCRESP2 >> 8;
    u->csd[11] = SDHCRESP2 & 0xFF;
    u->csd[12] = SDHCRESP3 >> 24;
    u->csd[13] = SDHCRESP3 >> 16;
    u->csd[14] = SDHCRESP3 >> 8;
    u->csd[15] = SDHCRESP3 & 0xFF;
    */
    u->csd[0]  = SDHCRESP3 >> 16;
    u->csd[1]  = SDHCRESP3 >> 8;
    u->csd[2]  = SDHCRESP3 & 0xFF;
    u->csd[3]  = SDHCRESP2 >> 24;
    u->csd[4]  = SDHCRESP2 >> 16;
    u->csd[5]  = SDHCRESP2 >> 8;
    u->csd[6]  = SDHCRESP2 & 0xFF;
    u->csd[7]  = SDHCRESP1 >> 24;
    u->csd[8]  = SDHCRESP1 >> 16;
    u->csd[9]  = SDHCRESP1 >> 8;
    u->csd[10] = SDHCRESP1 & 0xFF;
    u->csd[11] = SDHCRESP0 >> 24;
    u->csd[12] = SDHCRESP0 >> 16;
    u->csd[13] = SDHCRESP0 >> 8;
    u->csd[14] = SDHCRESP0 & 0xFF;
    u->csd[15] = 0;

    /* Disable the card. */
    sdhc_led(0);
    return 1;
}

/*
 * Get number of sectors on the disk.
 * Return nonzero if successful.
 */
static int card_size(int unit)
{
    struct disk *u = &sdhcdrives[unit];
    unsigned csize, n;
    int nsectors;

    if (! card_read_csd(unit))
        return 0;

    /* CSD register has different structure
     * depending upon protocol version. */
    switch (u->csd[0] >> 6) {
    case 1:                 /* SDC ver 2.00 */
        csize = u->csd[9] + (u->csd[8] << 8) + 1;
        nsectors = csize << 10;
        break;
    case 0:                 /* SDC ver 1.XX or MMC. */
        n = (u->csd[5] & 15) + ((u->csd[10] & 128) >> 7) +
            ((u->csd[9] & 3) << 1) + 2;
        csize = (u->csd[8] >> 6) + (u->csd[7] << 2) +
            ((u->csd[6] & 3) << 10) + 1;
        nsectors = csize << (n - 9);
        break;
    default:                /* Unknown version. */
        return 0;
    }
    printf("sdhc:  card size is %d MiB\n", nsectors / 2048);
    
    // use CMD_SELECT_CARD to move to transfer state
    int reply = card_cmd(unit, CMD_SELECT_CARD, u->rca, 0);
    if (reply != 0)
    {
        printf("sdhc:  CMD_SELECT_CARD failed, INTSTAT = %x\n", SDHCINTSTAT);
        sdhc_led(0);
        return 0;
    }
    int count;
    sdhc_wait_ready(100, &count);
    printf("sdhc:  sdhc_wait_ready returns, count = %d\n", count);
    
    return nsectors;
}

// wait for a word of sdhc response data to be ready
static inline void sdhc_wait_read_ready()
{
    while (!(SDHCINTSTAT & (1 << 5)));
    SDHCINTSTAT |= (1 << 5);
}

// wait for sdhc ready to received a word
static inline void sdhc_wait_write_ready()
{
    while (!(SDHCINTSTAT & (1 << 4)));
    SDHCINTSTAT |= (1 << 4);
}

// wait until a transfer is complete
static inline void sdhc_wait_transfer_complete()
{
    while (!(SDHCINTSTAT & (1 << 1)));
    SDHCINTSTAT |= (1 << 1);
}

// read one word of sdhc response data into an unsigned char array
static void sdhc_read_char(char *b)
{
    unsigned int shadow = SDHCDATA;
    *(b+3) = shadow >> 24;
    *(b+2) = shadow >> 16;
    *(b+1) = shadow >> 8;
    *(b+0) = shadow &  0xFF;
}

// write four bytes to one sdhc word
static void sdhc_write_char(char *b)
{
    unsigned int shadow = 0;
    shadow |= *(b+0);
    shadow |= *(b+1) << 8;
    shadow |= *(b+2) << 16;
    shadow |= *(b+3) << 24;
    SDHCDATA = shadow;
}

/*
 * Use CMD6 to enable high-speed mode.
 */
static void card_high_speed(int unit)
{
    int reply, i;
    struct disk *u = &sdhcdrives[unit];
    char status[64];

    /* Here we set HighSpeed 50MHz.
     * We do not tackle the power and io driver strength yet. */
    sdhc_led(1);
    printf("sdhc:  sending CMD_SWITCH_FUNC\n");
    reply = card_cmd(unit, CMD_SWITCH_FUNC, 0x80000001, 1);
    if (reply != 0) {
        /* Command timed out. */
        printf("sdhc:  card_size: SWITCH_FUNC timed out, reply = %d\n", reply);
        sdhc_led(0);
        return;
    }
    SDHCINTSTAT |= 1;
    printf("sdhc:  CMD_SWITCH_FUNC success, RESP0 = %x, reading data\n", SDHCRESP0);

    /* Read 64-byte status. */
    sdhc_wait_read_ready();
    for (i=0; i<64; i+=4)
        sdhc_read_char(&status[i]);
    printf("sdhc:  CMD_SWITCH_FUNC data read complete\n");
    for (i=0; i < 64; i+=8)
        printf("sdhc:  %2x %2x %2x %2x %2x %2x %2x %2x\n", 
            status[i], status[i+1], status[i+2], status[i+3], 
            status[i+4], status[i+5], status[i+6], status[i+7]);

    /* Do at least 8 _slow_ clocks to switch into the HS mode. */
    // can we just delay awhile??
    for (i=0; i<1000000; i++);

    if ((status[16] & 0xF) == 1) {
        /* The card has switched to high-speed mode. */
        int khz;

        //card_read_csd(unit);
        switch (u->csd[3]) {
        default:
            printf("sdhc:  Unknown speed csd[3] = %02x\n", u->csd[3]);
            /* fall through... */
        case TRANS_SPEED_25MHZ:
            /* 25 MHz - default clock for high speed mode. */
            khz = SDHC_FAST_KHZ;
            break;
        case TRANS_SPEED_50MHZ:
            /* 50 MHz - typical clock for SDHC cards. */
            khz = SDHC_FASTEST_KHZ;
            break;
        case TRANS_SPEED_100MHZ:
            printf("sdhc:  fast clock 100MHz\n");
            khz = SDHC_FASTEST_KHZ;
            break;
        case TRANS_SPEED_200MHZ:
            printf("sdhc:  fast clock 200MHz\n");
            khz = SDHC_FASTEST_KHZ;
            break;
        }
        sdhc_set_speed(khz);
        SDHCCON1 |= 4;
    }

    /* Save function group information for later use. */
    u->ma = status[0] << 8 | status[1];
    u->group[0] = status[12] << 8 | status[13];
    u->group[1] = status[10] << 8 | status[11];
    u->group[2] = status[8] << 8 | status[9];
    u->group[3] = status[6] << 8 | status[7];
    u->group[4] = status[4] << 8 | status[5];
    u->group[5] = status[2] << 8 | status[3];

    printf("sdhc:  function groups %x/%x/%x/%x/%x/%x",
        u->group[5] & 0x7fff, u->group[4] & 0x7fff,
        u->group[3] & 0x7fff, u->group[2] & 0x7fff,
        u->group[1] & 0x7fff, u->group[0] & 0x7fff);
    if (u->ma > 0)
        printf(", max current %u mA", u->ma);
    printf("\n");
    sdhc_led(0);
}

// use ACMD6 to enable 4-bit mode
static int
card_4bit(int unit)
{
    struct disk *u = &sdhcdrives[unit];
    int reply = card_cmd(unit, CMD_APP, u->rca, 0);   
    reply = card_cmd(unit, CMD_SET_BUS_WIDTH, 2, 0);
    if (0 != reply)
        return 1;
    SDHCCON1 |= 2;
    return 0;
}

/*
 * Read a block of data.
 * Return nonzero if successful.
 */
static int
card_read(int unit, unsigned int offset, char *data, unsigned int bcount)
{
    struct disk *u = &sdhcdrives[unit];
    int reply, i;
//printf("---R %s: unit = %d, blkno = %d, bcount = %d\n", __func__, unit, offset, bcount);

    /* Send read-multiple command. */
    sdhc_led(1);
    int cnt = 1;
    if (u->card_type != TYPE_SDHC)
        offset <<= 9;
    if (bcount >= SECTSIZE)
    {
        int rem = bcount % SECTSIZE;
        cnt = bcount / SECTSIZE;
        if (rem) cnt++;
    }
    reply = card_cmd(unit, cnt > 1 ? CMD_READ_MULTIPLE : CMD_READ_SINGLE, offset, cnt);
    if (reply != 0)
    {
        /* Command timed out. */
        printf("sdhc:  card_read: READ_MULTIPLE timed out, reply = %d\n", reply);
        sdhc_led(0);
        return 0;
    }
    SDHCINTSTAT |= 1;
    

    // Read data
    int base;
    for (base = 0; base < cnt; base++)
    {
        sdhc_wait_read_ready();
        for (i=0; i < SECTSIZE; i += 4)
            sdhc_read_char(&data[base * SECTSIZE + i]);
    }

    // Wait transfer complete
    sdhc_wait_transfer_complete();
    
    /*
    // dump the stuff we just read to the terminal
    for (i=0; i < bcount; i+=8)
        printf("sdhc:  %2x %2x %2x %2x %2x %2x %2x %2x\n", 
            data[i], data[i+1], data[i+2], data[i+3], 
            data[i+4], data[i+5], data[i+6], data[i+7]);
    */

    // Done!
    sdhc_led(0);
    return 1;
}

/*
 * Write a block of data.
 * Return nonzero if successful.
 */
static int
card_write(int unit, unsigned offset, char *data, unsigned bcount)
{
    struct disk *u = &sdhcdrives[unit];
    unsigned reply, i;
//printf("---W %s: unit = %d, blkno = %d, bcount = %d\n", __func__, unit, offset, bcount);

    int cnt = 1;
    if (bcount >= SECTSIZE)
    {
        int rem = bcount % SECTSIZE;
        cnt = bcount / SECTSIZE;
        if (rem) cnt++;
    }
    
    sdhc_led(1);

    // Send pre-erase count.
    card_cmd(unit, CMD_APP, u->rca, 0); 
    reply = card_cmd(unit, CMD_SET_WBECNT, cnt, 0);
    if (reply != 0)
    {
        // Command rejected.
        sdhc_led(0);
        printf("sdhc:  card_write: bad SET_WBECNT reply = %02x, count = %u\n",
            reply, (bcount + SECTSIZE - 1) / SECTSIZE);
        return 0;
    }

    // send write command
    if (u->card_type != TYPE_SDHC)
        offset <<= 9;
    reply = card_cmd(unit, cnt > 1 ? CMD_WRITE_MULTIPLE : CMD_WRITE_SINGLE, offset, cnt);
    if (reply != 0)
    {
        /* Command rejected. */
        sdhc_led(0);
        printf("sdhc:  card_write: bad WRITE_MULTIPLE reply = %02x\n", reply);
        return 0;
    }

    // write the data - hope it's a multiple of four bytes!
    int base;
    for (base = 0; base < cnt; base++)
    {
        sdhc_wait_write_ready();
        for (i = 0; i < SECTSIZE; i += 4)
            sdhc_write_char(&data[SECTSIZE * base + i]);
    }

    // wait for transfer complete
    sdhc_wait_transfer_complete();

    // Done!
    sdhc_led(0);
    return 1;
}

/*
 * Setup the SD card interface.
 * Get the card type and size.
 * Read a partition table.
 * Return 0 on failure.
 */
static int
sdhc_setup(struct disk *u)
{
    int unit = u->unit;

    // enable REFCLK4 which SDHC uses
    REFO4CON = 0;               // module off, no divisor
    REFO4CON |= 1 << 15;        // enable module

    // software reset the SD card
    //SDHCCON2 |= 1 << 24;
    //while (SDHCCON2 & 1 << 24);

    // power on the SD module
    SDHCCON1 |= 1 << 8;

    if (! card_init(unit)) {
        printf("sdhc:  no SD card detected\n");
        return 0;
    }
    /* Get the size of raw partition. */
    bzero(u->part, sizeof(u->part));
    u->part[RAWPART].dp_offset = 0;
    u->part[RAWPART].dp_size = card_size(unit);
    if (u->part[RAWPART].dp_size == 0) {
        printf("sdhc:  cannot get card size\n");
        return 0;
    }

    /* Switch to the high speed mode, if possible. */
    if (u->csd[4] & 0x40) {
        /* Class 10 card: switch to high-speed mode.
         * SPI interface of pic32 allows up to 25MHz clock rate. */
        card_high_speed(unit);
    }
    
    // switch to 4-bit SDHC mode
    if (0 != card_4bit(unit))
        printf("sdhc:  Could not switch to 4-bit mode\n");
    else
        printf("sdhc:  Switched to 4-bit mode\n");
    
    printf("sdhc:  type %s, size %u kbytes, speed %u Mbit/sec\n",
        u->card_type==TYPE_SDHC ? "SDHC" :
        u->card_type==TYPE_SD_II ? "II" : "I",
        u->part[RAWPART].dp_size / 2,
        SDHC_KHZ / 250);

    /* Read partition table. */
    u_int16_t buf[256];
    int s = splbio();
    if (! card_read(unit, 0, (char*)buf, sizeof(buf))) {
        splx(s);
        printf("sdhc:  cannot read partition table\n");
        return 0;
    }
    splx(s);
    if (buf[255] == MBR_MAGIC) {
        bcopy(&buf[223], &u->part[1], 64);
#if 1
        int i;
        for (i=1; i<=NPARTITIONS; i++) {
            if (u->part[i].dp_type != 0)
                printf("sdhc%d%c:  partition type %02x, sector %u, size %u kbytes\n",
                    unit, i+'a'-1, u->part[i].dp_type,
                    u->part[i].dp_offset,
                    u->part[i].dp_size / 2);
        }
#endif
    }
    return 1;
}

/*
 * Initialize a drive.
 */
int
sdhcopen(dev, flags, mode, p)
    dev_t dev;
    int flags, mode;
    struct proc *p;
{
    struct disk *u;
    int unit = sdhcunit(dev);
    int part = sdhcpart(dev);
    unsigned mask, i;

    if (unit >= NSDHC || part > NPARTITIONS)
        return ENXIO;
    u = &sdhcdrives[unit];
    u->unit = unit;

    /*
     * Setup the SD card interface.
     */
    if (u->part[RAWPART].dp_size == 0) {
        if (! sdhc_setup(u)) {
            return ENODEV;
        }
    }
    u->open++;

    /*
     * Warn if a partion is opened
     * that overlaps another partition which is open
     * unless one is the "raw" partition (whole disk).
     */
    mask = 1 << part;
    if (part != RAWPART && ! (u->openpart & mask)) {
        unsigned start = u->part[part].dp_offset;
        unsigned end = start + u->part[part].dp_size;

        /* Check for overlapped partitions. */
        for (i=0; i<=NPARTITIONS; i++) {
            struct diskpart *pp = &u->part[i];

            if (i == part || i == RAWPART)
                continue;

            if (pp->dp_offset + pp->dp_size <= start ||
                pp->dp_offset >= end)
                continue;

            if (u->openpart & (1 << i))
                log(LOG_WARNING, "sd%d%c: overlaps open partition (sd%d%c)\n",
                    unit, part + 'a' - 1,
                    unit, pp - u->part + 'a' - 1);
        }
    }

    u->openpart |= mask;
    switch (mode) {
    case S_IFCHR:
        u->copenpart |= mask;
        break;
    case S_IFBLK:
        u->bopenpart |= mask;
        break;
    }
    return 0;
}

/*
 * Read/write routine for a buffer.  Finds the proper unit, range checks
 * arguments, and schedules the transfer.  Does not wait for the transfer
 * to complete.  Multi-page transfers are supported.  All I/O requests must
 * be a multiple of a sector in length.
 */
void
sdhcstrategy(bp)
    struct buf *bp;
{
    struct disk *u;    /* Disk unit to do the IO.  */
    int unit = sdhcunit(bp->b_dev);
    int s;
    unsigned offset;
//printf("%s: unit = %d, blkno = %d, bcount = %d\n", __func__, unit, bp->b_blkno, bp->b_bcount);

    if (unit >= NSDHC || bp->b_blkno < 0) {
        printf("sdhcstrategy: unit = %d, blkno = %d, bcount = %d\n",
            unit, bp->b_blkno, bp->b_bcount);
        bp->b_error = EINVAL;
        goto bad;
    }
    u = &sdhcdrives[unit];
    offset = bp->b_blkno;
    if (u->open) {
        /*
         * Determine the size of the transfer, and make sure it is
         * within the boundaries of the partition.
         */
        struct diskpart *p = &u->part[sdhcpart(bp->b_dev)];
        long maxsz = p->dp_size;
        long sz = (bp->b_bcount + DEV_BSIZE - 1) >> DEV_BSHIFT;

        offset += p->dp_offset;
//printf("%s: sdpart=%u, offset=%u, maxsz=%u, sz=%u\n", __func__, sdpart(bp->b_dev), offset, maxsz, sz);
        if (offset == 0 &&
            ! (bp->b_flags & B_READ) && ! u->wlabel) {
                /* Write to partition table not allowed. */
                bp->b_error = EROFS;
                goto bad;
        }
        if (bp->b_blkno + sz > maxsz) {
                /* if exactly at end of disk, return an EOF */
                if (bp->b_blkno == maxsz) {
                        bp->b_resid = bp->b_bcount;
                        biodone(bp);
//printf("%s: done EOF\n", __func__);
                        return;
                }
                /* or truncate if part of it fits */
                sz = maxsz - bp->b_blkno;
                if (sz <= 0) {
                        bp->b_error = EINVAL;
                        goto bad;
                }
                bp->b_bcount = sz << DEV_BSHIFT;
        }
    } else {
        /* Reading the partition table. */
//printf("%s: reading the partition table\n", __func__);
        offset = 0;
    }
    if (u->dkindex >= 0) {
        /* Update disk statistics. */
        dk_busy |= 1 << u->dkindex;
        dk_xfer[u->dkindex]++;
        dk_wds[u->dkindex] += bp->b_bcount >> 6;
    }

    s = splbio();
    if (bp->b_flags & B_READ) {
        card_read(unit, offset, bp->b_un.b_addr, bp->b_bcount);
    } else {
        card_write(unit, offset, bp->b_un.b_addr, bp->b_bcount);
    }
    biodone(bp);
    splx(s);
//printf("%s: done OK\n", __func__);

    if (u->dkindex >= 0)
        dk_busy &= ~(1 << u->dkindex);
    return;

bad:
    bp->b_flags |= B_ERROR;
    biodone(bp);
//printf("%s: failed \n", __func__);
}

int
sdhcsize(dev)
    dev_t dev;
{
    int unit = sdhcunit(dev);
    int part = sdhcpart(dev);
    struct disk *u = &sdhcdrives[unit];

    if (unit >= NSDHC || part > NPARTITIONS)
        return -1;

    /*
     * Setup the SD card interface, if not done yet.
     */
    if (u->part[RAWPART].dp_size == 0) {
        if (! sdhc_setup(u)) {
            return -1;
        }
    }
    return u->part[part].dp_size;
}

int
sdhcioctl(dev, cmd, data, flag, p)
    dev_t dev;
    u_long cmd;
    caddr_t data;
    int flag;
    struct proc *p;
{
    int unit = sdhcunit(dev);
    int part = sdhcpart(dev);
    struct diskpart *pp;
    int error = 0;

    switch (cmd) {

    case DIOCGETPART:
        /* Get partition table entry. */
        pp = &sdhcdrives[unit].part[part];
//printf("--- %s: DIOCGETPART unit = %d, part = %d, type = %u, size = %u\n", __func__, unit, part, pp->dp_type, pp->dp_size);
        *(struct diskpart*) data = *pp;
        break;

    default:
        error = ENOTTY;
        break;
    }
    return error;
}

/*
 * Non-interrupt driven, non-dma dump routine.
 */
int
sdhcdump(dev)
    dev_t dev;
{
    // TODO
    return ENXIO;
}

int
sdhcread(dev, uio)
    dev_t dev;
    struct uio *uio;
{
    return physio(sdhcstrategy, 0, dev, B_READ, minphys, uio);
}

int
sdhcwrite(dev, uio)
    dev_t dev;
    struct uio *uio;
{
    return physio(sdhcstrategy, 0, dev, B_WRITE, minphys, uio);
}

/*
 * Test to see if device is present.
 * Return true if found and initialized ok.
 */
static int
sdhcprobe(config)
    struct conf_device *config;
{
    int unit = config->dev_unit;
    struct disk *u = &sdhcdrives[unit];

    if (unit < 0 || unit >= NSDHC)
        return 0;

    if (sdhc_setup(u) != 1) {
        printf("sdhc: cannot open SPI%u port\n", config->dev_ctlr);
        return 0;
    }

    //sdhc_set_speed(250);

    /* Assign disk index. */
    if (dk_ndrive < DK_NDRIVE) {
        u->dkindex = dk_ndrive++;

        /* Estimated transfer rate in 16-bit words per second. */
        dk_wpms[u->dkindex] = SDHC_KHZ / 8;
    } else
        u->dkindex = -1;

    /* Configure LED pin as output. */
#ifdef SD_LED_PORT
    ANSEL_CLR(SD_LED_PORT) = 1 << SD_LED_PIN;
    TRIS_CLR(SD_LED_PORT) = 1 << SD_LED_PIN;
#endif
    return 1;
}

struct driver sdhcdriver = {
    "sdhc", sdhcprobe,
};
#endif
