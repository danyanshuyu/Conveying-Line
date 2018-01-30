/********************************** (C) COPYRIGHT *******************************
* File Name          : CH563.C
* Author             : WCH
* Version            : V1.0
* Date               : 2013/11/15
* Description        : FLASH_ROM_AND_EEPROM
*******************************************************************************/

/* Flash-ROM feature:
     for store program code, support block erasing, dword and page writing, dword verifying, unit for Length is byte,
     minimal quantity for write or verify is one dword (4-bytes),
     256 bytes/page for writing, FLASH_ROM_WRITE support one dword or more dword writing, but multiple of 256 is the best,
     4KB (4096 bytes) bytes/block for erasing, so multiple of 4096 is the best */

/* EEPROM feature:
     for store data, support block erasing, byte and page writing, byte reading,
     minimal quantity for write or read is one byte,
     256 bytes/page for writing, EEPROM_WRITE support one byte or more byte writing, but multiple of 256 is the best;
     4KB (4096 bytes) bytes/block for erasing, so multiple of 4096 is the best */

#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE    256                       // Flash-ROM & EEPROM page size for writing
#define EEPROM_BLOCK_SIZE   4096                      // Flash-ROM & EEPROM block size for erasing			
#define EEPROM_MIN_ER_SIZE  EEPROM_BLOCK_SIZE         // Flash-ROM & EEPROM minimal size for erasing
#define EEPROM_MIN_WR_SIZE  1                         // EEPROM minimal size for writing
#define EEPROM_MAX_SIZE     0x7000                    // EEPROM maximum size,28KB
#endif
#define FLASH_MIN_WR_SIZE   4                         // Flash-ROM minimal size for writing
#define FLASH_ROM_MAX_SIZE  0x038000                  // Flash-ROM maximum program size, 224KB
#define FLASH_ROM_LOCK_AUTO 0x50                      // auto LockFlag for Flash-ROM

extern VOID GET_UNIQUE_ID( PVOID Buffer );            // get 64 bit unique ID

extern VOID FLASH_ROM_PWR_DOWN( VOID );               // power-down FlashROM

extern VOID FLASH_ROM_PWR_UP( VOID );                 // power-up FlashROM

extern UINT8 CH563_EEPROM_READ( UINT32 StartAddr, PVOID Buffer, UINT32 Length );  // read EEPROM data block,return 0 if success

extern UINT8 CH563_EEPROM_ERASE( UINT32 StartAddr, UINT32 Length );  // erase EEPROM block, minimal block is 4KB, return 0 if success

extern UINT8 CH563_EEPROM_WRITE( UINT32 StartAddr, PVOID Buffer, UINT32 Length );  // write EEPROM data block, return 0 if success

