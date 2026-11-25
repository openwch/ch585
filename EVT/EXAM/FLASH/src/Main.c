/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : FLASH读写例程
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH58x_common.h"

#define WRProt_Size    0x08  /* 代码保护大小，单位4KB */

/* -------------------- Data-Flash(EEPROM) 操作地址与长度 -------------------- */
#define EEPROM_TEST_ADDR        (0x0000)                /* Data-Flash 操作起始地址 */
#define EEPROM_TEST_LEN         (500)                   /* Data-Flash 读写长度 */
#define EEPROM_ERASE_LEN        (EEPROM_BLOCK_SIZE)     /* Data-Flash 擦除长度(EEPROM_BLOCK_SIZE=4096) */

/* -------------------- Flash-ROM(Code Flash) 操作地址与长度 -------------------- */
#define FLASH_ROM_TEST_ADDR     (20 * 1024)             /* Flash-ROM 操作起始地址 */
#define FLASH_ROM_TEST_LEN      (128)                   /* Flash-ROM 读写/校验长度，需为4字节整数倍 */
#define FLASH_ROM_ERASE_LEN     (4096)                  /* Flash-ROM 擦除长度 */

/* -------------------- 可操作地址范围 -------------------- */
/* Data-Flash 可操作范围: [EEPROM_RANGE_BASE, EEPROM_RANGE_END) */
#define EEPROM_RANGE_BASE       (0x0000)
#define EEPROM_RANGE_END        (0x8000)               

/* Flash-ROM 可操作范围: [FLASH_ROM_RANGE_BASE, FLASH_ROM_RANGE_END)*/
#define FLASH_ROM_RANGE_BASE    (20 * 1024)
#define FLASH_ROM_RANGE_END     (0x70000)               

uint8_t TestBuf[1024];

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   调试初始化
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOA_SetBits(GPIO_Pin_14);
    GPIOPinRemap(ENABLE, RB_PIN_UART0);
    GPIOA_ModeCfg(GPIO_Pin_14, GPIO_ModeOut_PP_5mA);
    UART0_DefInit();
}

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main(void)
{
    uint16_t i;
    uint8_t  s;

    /* Flash 操作地址与长度，由宏定义赋值给变量 */
    uint32_t eeprom_addr   = EEPROM_TEST_ADDR;
    uint32_t eeprom_len    = EEPROM_TEST_LEN;
    uint32_t eeprom_er_len = EEPROM_ERASE_LEN;
    uint32_t rom_addr      = FLASH_ROM_TEST_ADDR;
    uint32_t rom_len       = FLASH_ROM_TEST_LEN;
    uint32_t rom_er_len    = FLASH_ROM_ERASE_LEN;

    HSECFG_Capacitance(HSECap_18p);
    SetSysClock(SYSCLK_FREQ);

    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);

#if 1 // 读写Data-Flash

    /* 第一次操作前，判断 Data-Flash 操作地址范围是否超出可操作范围 */
    if((eeprom_addr < EEPROM_RANGE_BASE) ||
       ((eeprom_addr + eeprom_len) > EEPROM_RANGE_END) ||
       ((eeprom_addr + eeprom_er_len) > EEPROM_RANGE_END))
    {
        PRINT("Data-Flash operation out of range!\n");
        while(1);
    }

    PRINT("EEPROM_READ...\n");
    EEPROM_READ(eeprom_addr, TestBuf, eeprom_len);
    for(i = 0; i < eeprom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");
    // Data-Flash minimal size for erasing is EEPROM_PAGE_SIZE
    s = EEPROM_ERASE(eeprom_addr, eeprom_er_len);
    PRINT("EEPROM_ERASE=%02x\n", s);
    PRINT("EEPROM_READ...\n");
    EEPROM_READ(eeprom_addr, TestBuf, eeprom_len);
    for(i = 0; i < eeprom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    for(i = 0; i < eeprom_len; i++)
        TestBuf[i] = 0x0 + i;
    s = EEPROM_WRITE(eeprom_addr, TestBuf, eeprom_len);
    PRINT("EEPROM_WRITE=%02x\n", s);
    PRINT("EEPROM_READ...\n");
    EEPROM_READ(eeprom_addr, TestBuf, eeprom_len);
    for(i = 0; i < eeprom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

#endif

#if 1 // 获取唯一ID，MAC地址，读写Flash-ROM

    PRINT("GET_UNIQUE_ID...\n");
    GET_UNIQUE_ID(TestBuf);
    for(i = 0; i < 8; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    PRINT("FlashMACADDRESS...\n");
    GetMACAddress(TestBuf);
    for(i = 0; i < 6; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    /* 第一次操作前，判断 Flash-ROM 操作地址范围是否超出可操作范围 */
    if((rom_addr < FLASH_ROM_RANGE_BASE) ||
       ((rom_addr + rom_len) > FLASH_ROM_RANGE_END) ||
       ((rom_addr + rom_er_len) > FLASH_ROM_RANGE_END))
    {
        PRINT("Flash-ROM operation out of range!\n");
        while(1);
    }

    PRINT("FLASH_READ 20k...\n");
    FLASH_ROM_READ(rom_addr, TestBuf, rom_len);
    for(i = 0; i < rom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    s = FLASH_ROM_ERASE(rom_addr, rom_er_len);
    PRINT("FLASH_ROM_ERASE=%02x\n", s);
    PRINT("FLASH_READ 20k...\n");
    FLASH_ROM_READ(rom_addr, TestBuf, rom_len);
    for(i = 0; i < rom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

    for(i = 0; i < rom_len; i++)
    {
        TestBuf[i] = 0x70 + i;
    }
    s = FLASH_ROM_WRITE(rom_addr, TestBuf, rom_len);//Codeflash写入必须是4字节的整数倍
    PRINT("FlashWrite=%02x\n", s);
    s = FLASH_ROM_VERIFY(rom_addr, TestBuf, rom_len);
    PRINT("FlashVerify=%02x\n", s);

    PRINT("FLASH_READ...\n");
    FLASH_ROM_READ(rom_addr, TestBuf, rom_len);
    for(i = 0; i < rom_len; i++)
    {
        PRINT("%02x ", TestBuf[i]);
    }
    PRINT("\n");

#endif

#if 0 /* 修改用户配置值 */
    s = UserOptionByteConfig(ENABLE, ENABLE, ENABLE, WRProt_Size);
    if(s)
        PRINT("ERR\n");
    else
    {
        PRINT("suc\n");
        mDelaymS(10);
        UserOptionByte_Active();
    }
#endif

#if 0 /* 关闭两线调试接口 */
    s = UserOptionByteClose_SWD();
    if(s)
        PRINT("ERR\n");
    else
    {
        PRINT("suc\n");
        mDelaymS(10);
        UserOptionByte_Active();
    }
#endif

    while(1);
}
