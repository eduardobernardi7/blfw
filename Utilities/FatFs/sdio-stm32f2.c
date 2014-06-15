/*-----------------------------------------------------------------------*/
/* MMC/SDC (in SPI mode) control module  (C)ChaN, 2007                   */
/*-----------------------------------------------------------------------*/
/* Only rcvr_spi(), xmit_spi(), disk_timerproc() and some macros         */
/* are platform dependent.                                               */
/*-----------------------------------------------------------------------*/

/*
* This file was modified from a sample available from the FatFs
* web site. It was modified to work with a Stellaris EK-LM3S6965
* evaluation board.
*
* Note that the SSI port is shared with the osram display. The code
* in this file does not attempt to share the SSI port with the osram,
* it assumes the osram is not being used.
*/


#include "stm32f2xx.h"
#include "stm322xg_eval_sdio_sd.h"
#include <string.h> 
#include "diskio.h"

/* Definitions for MMC/SDC command */
#define CMD0 0x40	// software reset
#define CMD1 0x41	// brings card out of idle state
#define CMD2 0x42	// not used in SPI mode
#define CMD3 0x43	// not used in SPI mode
#define CMD4 0x44	// not used in SPI mode
#define CMD5 0x45	// Reserved
#define CMD6 0x46	// Reserved
#define CMD7 0x47	// not used in SPI mode
#define CMD8 0x48	// Reserved
#define CMD9 0x49	// ask card to send card speficic data (CSD)
#define CMD10 0x4A	// ask card to send card identification (CID)
#define CMD11 0x4B	// not used in SPI mode
#define CMD12 0x4C	// stop transmission on multiple block read
#define CMD13 0x4D	// ask the card to send it's status register
#define CMD14 0x4E	// Reserved
#define CMD15 0x4F	// not used in SPI mode
#define CMD16 0x50	// sets the block length used by the memory card
#define CMD17 0x51	// read single block
#define CMD18 0x52	// read multiple block
#define CMD19 0x53	// Reserved
#define CMD20 0x54	// not used in SPI mode
#define CMD21 0x55	// Reserved
#define CMD22 0x56	// Reserved
#define CMD23 0x57	// Reserved
#define CMD24 0x58	// writes a single block
#define CMD25 0x59	// writes multiple blocks
#define CMD26 0x5A	// not used in SPI mode
#define CMD27 0x5B	// change the bits in CSD
#define CMD28 0x5C	// sets the write protection bit
#define CMD29 0x5D	// clears the write protection bit
#define CMD30 0x5E	// checks the write protection bit
#define CMD31 0x5F	// Reserved
#define CMD32 0x60	// Sets the address of the first sector of the erase group
#define CMD33 0x61	// Sets the address of the last sector of the erase group
#define CMD34 0x62	// removes a sector from the selected group
#define CMD35 0x63	// Sets the address of the first group
#define CMD36 0x64	// Sets the address of the last erase group
#define CMD37 0x65	// removes a group from the selected section
#define CMD38 0x66	// erase all selected groups
#define CMD39 0x67	// not used in SPI mode
#define CMD40 0x68	// not used in SPI mode
#define CMD41 0x69	// Reserved
#define CMD42 0x6A	// locks a block
// CMD43 ... CMD57 are Reserved
#define CMD58 0x7A	// reads the OCR register
#define CMD59 0x7B	// turns CRC off

//#define CMD0    (0x40+0)    /* GO_IDLE_STATE */
//#define CMD1    (0x40+1)    /* SEND_OP_COND */
//#define CMD8    (0x40+8)    /* SEND_IF_COND */
//#define CMD9    (0x40+9)    /* SEND_CSD */
//#define CMD10    (0x40+10)    /* SEND_CID */
//#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
//#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
//#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
//#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
//#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
//#define CMD24    (0x40+24)    /* WRITE_BLOCK */
//#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
//#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
//#define CMD58    (0x40+58)    /* READ_OCR */


#define BLOCK_SIZE            512 /* Block Size in Bytes */

static volatile
DSTATUS Stat = STA_NOINIT;    /* Disk status */

static volatile
BYTE Timer1, Timer2;    /* 100Hz decrement timer */




/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (    BYTE drv        /* Physical drive nmuber (0) */
                         )
{
  SD_Error  Status;
  
  //	printf("disk_initialize %d\n", drv);
  
  /* Supports only single drive */
  if (drv)
  {
    return STA_NOINIT;
  }
  /*-------------------------- SD Init ----------------------------- */
  Status = SD_Init();
  if (Status!=SD_OK )
  {
    //					puts("Initialization Fail");
    return STA_NOINIT;
    
  }
  else
  {
    
    return RES_OK;
  }

}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
                     BYTE drv        /* Physical drive nmuber (0) */
                       )
{
  
#if(1)
  DSTATUS stat = 0;
  
  if (SD_Detect() != SD_PRESENT)
    stat |= STA_NODISK;
  
  // STA_NOTINIT - Subsystem not initailized
  // STA_PROTECTED - Write protected, MMC/SD switch if available
  
  return(stat);
#endif
  
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
                   BYTE drv,            /* Physical drive nmuber (0) */
                   BYTE *buff,            /* Pointer to the data buffer to store read data */
                   DWORD sector,        /* Start sector number (LBA) */
                   BYTE count            /* Sector count (1..255) */
                     )
{
  
  SD_Error Status;
  
  if (SD_Detect() != SD_PRESENT)
    return(RES_NOTRDY);
  
  if ((DWORD)buff & 3) // DMA Alignment failure, do single up to aligned buffer
  {
    DRESULT res = RES_OK;
    DWORD scratch[BLOCK_SIZE / 4]; // Alignment assured, you'll need a sufficiently big stack
    
    while(count--)
    {
      res = disk_read(drv, (void *)scratch, sector++, 1);
      
      if (res != RES_OK)
        break;
      
      memcpy(buff, scratch, BLOCK_SIZE);
      
      buff += BLOCK_SIZE;
    }
    
    return(res);
  }
  
  Status = SD_ReadMultiBlocks(buff, sector, BLOCK_SIZE, count); // 4GB Compliant
  
  if (Status == SD_OK)
  {
    SDTransferState State;
    
    Status = SD_WaitReadOperation(); // Check if the Transfer is finished
    
    while((State = SD_GetStatus()) == SD_TRANSFER_BUSY); // BUSY, OK (DONE), ERROR (FAIL)
    
    if ((State == SD_TRANSFER_ERROR) || (Status != SD_OK))
      return(RES_ERROR);
    else
      return(RES_OK);
  }
  else
    return(RES_ERROR);

}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write (
                    BYTE drv,            /* Physical drive nmuber (0) */
                    const BYTE *buff,    /* Pointer to the data to be written */
                    DWORD sector,        /* Start sector number (LBA) */
                    BYTE count            /* Sector count (1..255) */
                      )
{

  SD_Error Status;
  
  //	printf("disk_write %d %p %10d %d\n",drv,buff,sector,count);
  
  if (SD_Detect() != SD_PRESENT)
    return(RES_NOTRDY);
  
  if ((DWORD)buff & 3) // DMA Alignment failure, do single up to aligned buffer
  {
    DRESULT res = RES_OK;
    DWORD scratch[BLOCK_SIZE / 4]; // Alignment assured, you'll need a sufficiently big stack
    
    while(count--)
    {
      memcpy(scratch, buff, BLOCK_SIZE);
      
      res = disk_write(drv, (void *)scratch, sector++, 1);
      
      if (res != RES_OK)
        break;
      
      buff += BLOCK_SIZE;
    }
    
    return(res);
  }
  
  Status = SD_WriteMultiBlocks((uint8_t *)buff, sector, BLOCK_SIZE, count); // 4GB Compliant
  
  if (Status == SD_OK)
  {
    SDTransferState State;
    
    Status = SD_WaitWriteOperation(); // Check if the Transfer is finished
    
    while((State = SD_GetStatus()) == SD_TRANSFER_BUSY); // BUSY, OK (DONE), ERROR (FAIL)
    
    if ((State == SD_TRANSFER_ERROR) || (Status != SD_OK))
      return(RES_ERROR);
    else
      return(RES_OK);
  }
  else
    return(RES_ERROR);
 
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
                    BYTE drv,        /* Physical drive nmuber (0) */
                    BYTE ctrl,        /* Control code */
                    void *buff        /* Buffer to send/receive control data */
                      )
{
  return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Device Timer Interrupt Procedure  (Platform dependent)                */
/*-----------------------------------------------------------------------*/
/* This function must be called in period of 10ms                        */

void disk_timerproc (void)
{
  //    BYTE n, s;
  BYTE n;
  
  
  n = Timer1;                        /* 100Hz decrement timer */
  if (n) Timer1 = --n;
  n = Timer2;
  if (n) Timer2 = --n;
  
}

/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support a real time clock.          */

DWORD get_fattime (void)
{
  return    ((2007UL-1980) << 25)    // Year = 2007
    | (6UL << 21)            // Month = June
      | (5UL << 16)            // Day = 5
        | (11U << 11)            // Hour = 11
          | (38U << 5)            // Min = 38
            | (0U >> 1)                // Sec = 0
              ;
  
}
