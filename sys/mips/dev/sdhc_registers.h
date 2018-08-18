// This file contains register definitions for the PIC32MZ DA SDHC peripheral.

#include <machine/pic32mz.h>
 
// general format:
// #define REG_NAME PIC32_R(lowest 5 nibbles of address)

#define SDHCBLKCON      PIC32_R(0xEC004)
#define SDHCARG         PIC32_R(0xEC008)
#define SDHCMODE        PIC32_R(0xEC00C)
#define SDHCRESP0       PIC32_R(0xEC010)
#define SDHCRESP1       PIC32_R(0xEC014)
#define SDHCRESP2       PIC32_R(0xEC018)
#define SDHCRESP3       PIC32_R(0xEC01C)
#define SDHCDATA        PIC32_R(0xEC020)
#define SDHCSTAT1       PIC32_R(0xEC024)
#define SDHCCON1        PIC32_R(0xEC028)
#define SDHCCON2        PIC32_R(0xEC02C)
#define SDHCINTSTAT     PIC32_R(0xEC030)
#define SDHCINTEN       PIC32_R(0xEC034)
#define SDHCINTSEN      PIC32_R(0xEC038)
#define SDHCSTAT2       PIC32_R(0xEC03C)
#define SDHCCAP         PIC32_R(0xEC040)
#define SDHCMAXCAP      PIC32_R(0xEC048)
#define SDHCFE          PIC32_R(0xEC050)
#define SDHCAESTAT      PIC32_R(0xEC054)
#define SDHCAADDR       PIC32_R(0xEC058)        
