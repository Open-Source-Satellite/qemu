/*
 * ARM V2M MPS2 board emulation.
 *
 * Copyright (c) 2017 Linaro Limited
 * Written by Peter Maydell
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 or
 *  (at your option) any later version.
 */

/* The MPS2 and MPS2+ dev boards are FPGA based (the 2+ has a bigger
 * FPGA but is otherwise the same as the 2). Since the CPU itself
 * and most of the devices are in the FPGA, the details of the board
 * as seen by the guest depend significantly on the FPGA image.
 * We model the following FPGA images:
 *  "mps2-an385" -- Cortex-M3 as documented in ARM Application Note AN385
 *  "mps2-an386" -- Cortex-M4 as documented in ARM Application Note AN386
 *  "mps2-an500" -- Cortex-M7 as documented in ARM Application Note AN500
 *  "mps2-an511" -- Cortex-M3 'DesignStart' as documented in AN511
 *
 * Links to the TRM for the board itself and to the various Application
 * Notes which document the FPGA images can be found here:
 *   https://developer.arm.com/products/system-design/development-boards/cortex-m-prototyping-system
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/arm/boot.h"
#include "hw/arm/armv7m.h"
#include "hw/or-irq.h"
#include "hw/boards.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "hw/misc/unimp.h"
#include "hw/char/cmsdk-apb-uart.h"
#include "hw/timer/cmsdk-apb-timer.h"
#include "hw/timer/cmsdk-apb-dualtimer.h"
#include "hw/misc/mps2-scc.h"
#include "hw/misc/mps2-fpgaio.h"
#include "hw/ssi/pl022.h"
#include "hw/i2c/arm_sbcon_i2c.h"
#include "hw/net/lan9118.h"
#include "net/net.h"
#include "hw/watchdog/cmsdk-apb-watchdog.h"
#include "qom/object.h"
#include "hw/char/stm32f2xx_usart.h"
#include "hw/arm/stm32f7xx_pwr.h"
#include "hw/arm/stm32f7xx_rcc.h"

#define STM_NUM_USARTS 6

typedef enum MPS2FPGAType {
    FPGA_AN385,
    FPGA_AN386,
    FPGA_AN500,
    FPGA_AN511,
} MPS2FPGAType;

struct MPS2MachineClass {
    MachineClass parent;
    MPS2FPGAType fpga_type;
    uint32_t scc_id;
    bool has_block_ram;
    hwaddr ethernet_base;
    hwaddr psram_base;
};

struct MPS2MachineState {
    MachineState parent;

    ARMv7MState armv7m;
    MemoryRegion ssram1;
    MemoryRegion ssram1_m;
    MemoryRegion ssram23;
    MemoryRegion ssram23_m;
    MemoryRegion blockram;
    MemoryRegion blockram_m1;
    MemoryRegion blockram_m2;
    MemoryRegion blockram_m3;
    MemoryRegion sram;
    /* FPGA APB subsystem */
    MPS2SCC scc;
    MPS2FPGAIO fpgaio;
    /* CMSDK APB subsystem */
    CMSDKAPBDualTimer dualtimer;
    CMSDKAPBWatchdog watchdog;

    /* STM32 hardware */
    STM32F2XXUsartState usart[STM_NUM_USARTS];
    STM32F7XXPwrState   pwr;
    Stm32f7xxRcc        rcc;
};

#define TYPE_MPS2_MACHINE "mps2"
#define TYPE_MPS2_AN385_MACHINE MACHINE_TYPE_NAME("mps2-an385")
#define TYPE_MPS2_AN386_MACHINE MACHINE_TYPE_NAME("mps2-an386")
//#define TYPE_MPS2_AN500_MACHINE MACHINE_TYPE_NAME("mps2-an500")
#define TYPE_MPS2_AN500_MACHINE MACHINE_TYPE_NAME("stm32h753-nucleo")
#define TYPE_MPS2_AN511_MACHINE MACHINE_TYPE_NAME("mps2-an511")

OBJECT_DECLARE_TYPE(MPS2MachineState, MPS2MachineClass, MPS2_MACHINE)

/* Main SYSCLK frequency in Hz */
#define SYSCLK_FRQ 25000000

/* Initialize the auxiliary RAM region @mr and map it into
 * the memory map at @base.
 */
static void make_ram(MemoryRegion *mr, const char *name,
                     hwaddr base, hwaddr size)
{
    memory_region_init_ram(mr, NULL, name, size, &error_fatal);
    memory_region_add_subregion(get_system_memory(), base, mr);
}

/* Create an alias of an entire original MemoryRegion @orig
 * located at @base in the memory map.
 */
static void make_ram_alias(MemoryRegion *mr, const char *name,
                           MemoryRegion *orig, hwaddr base)
{
    memory_region_init_alias(mr, NULL, name, orig, 0,
                             memory_region_size(orig));
    memory_region_add_subregion(get_system_memory(), base, mr);
}

static void mps2_common_init(MachineState *machine)
{
    MPS2MachineState *mms = MPS2_MACHINE(machine);
    MPS2MachineClass *mmc = MPS2_MACHINE_GET_CLASS(machine);
    MemoryRegion *system_memory = get_system_memory();
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    DeviceState *armv7m, *sccdev, *usart, *pwr, *rcc;
    int i;
    SysBusDevice *busdev;
    //CPUARMState *env;
    
    
    if (strcmp(machine->cpu_type, mc->default_cpu_type) != 0) {
        error_report("This board can only be used with CPU %s",
                     mc->default_cpu_type);
        exit(1);
    }

    if (machine->ram_size != mc->default_ram_size) {
        char *sz = size_to_str(mc->default_ram_size);
        error_report("Invalid RAM size, should be %s", sz);
        g_free(sz);
        exit(EXIT_FAILURE);
    }

    /* The FPGA images have an odd combination of different RAMs,
     * because in hardware they are different implementations and
     * connected to different buses, giving varying performance/size
     * tradeoffs. For QEMU they're all just RAM, though. We arbitrarily
     * call the 16MB our "system memory", as it's the largest lump.
     *
     * AN385/AN386/AN511:
     *  0x21000000 .. 0x21ffffff : PSRAM (16MB)
     * AN385/AN386/AN500:
     *  0x00000000 .. 0x003fffff : ZBT SSRAM1
     *  0x00400000 .. 0x007fffff : mirror of ZBT SSRAM1
     *  0x20000000 .. 0x203fffff : ZBT SSRAM 2&3
     *  0x20400000 .. 0x207fffff : mirror of ZBT SSRAM 2&3
     * AN385/AN386 only:
     *  0x01000000 .. 0x01003fff : block RAM (16K)
     *  0x01004000 .. 0x01007fff : mirror of above
     *  0x01008000 .. 0x0100bfff : mirror of above
     *  0x0100c000 .. 0x0100ffff : mirror of above
     * AN511 only:
     *  0x00000000 .. 0x0003ffff : FPGA block RAM
     *  0x00400000 .. 0x007fffff : ZBT SSRAM1
     *  0x20000000 .. 0x2001ffff : SRAM
     *  0x20400000 .. 0x207fffff : ZBT SSRAM 2&3
     * AN500 only:
     *  0x60000000 .. 0x60ffffff : PSRAM (16MB)
     *
     * The AN385/AN386 has a feature where the lowest 16K can be mapped
     * either to the bottom of the ZBT SSRAM1 or to the block RAM.
     * This is of no use for QEMU so we don't implement it (as if
     * zbt_boot_ctrl is always zero).
     */
    memory_region_add_subregion(system_memory, mmc->psram_base, machine->ram);

    if (mmc->has_block_ram) {
        make_ram(&mms->blockram, "mps.blockram", 0x01000000, 0x4000);
        make_ram_alias(&mms->blockram_m1, "mps.blockram_m1",
                       &mms->blockram, 0x01004000);
        make_ram_alias(&mms->blockram_m2, "mps.blockram_m2",
                       &mms->blockram, 0x01008000);
        make_ram_alias(&mms->blockram_m3, "mps.blockram_m3",
                       &mms->blockram, 0x0100c000);
    }

    switch (mmc->fpga_type) {
    case FPGA_AN385:
    case FPGA_AN386:
    case FPGA_AN500:
        make_ram(&mms->ssram1, "mps.ssram1", 0x0, 0x400000);
        make_ram_alias(&mms->ssram1_m, "mps.ssram1_m", &mms->ssram1, 0x400000);
        make_ram(&mms->ssram23, "mps.ssram23", 0x20000000, 0x400000);
        make_ram_alias(&mms->ssram23_m, "mps.ssram23_m",
                       &mms->ssram23, 0x20400000);
        break;
    case FPGA_AN511:
        make_ram(&mms->blockram, "mps.blockram", 0x0, 0x40000);
        make_ram(&mms->ssram1, "mps.ssram1", 0x00400000, 0x00800000);
        make_ram(&mms->sram, "mps.sram", 0x20000000, 0x20000);
        make_ram(&mms->ssram23, "mps.ssram23", 0x20400000, 0x400000);
        break;
    default:
        g_assert_not_reached();
    }

    object_initialize_child(OBJECT(mms), "armv7m", &mms->armv7m, TYPE_ARMV7M);
    
    armv7m = DEVICE(&mms->armv7m);
    switch (mmc->fpga_type) {
    case FPGA_AN385:
    case FPGA_AN386:
    case FPGA_AN500:
        qdev_prop_set_uint32(armv7m, "num-irq", 32);
        break;
    case FPGA_AN511:
        qdev_prop_set_uint32(armv7m, "num-irq", 64);
        break;
    default:
        g_assert_not_reached();
    }
    qdev_prop_set_string(armv7m, "cpu-type", machine->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&mms->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&mms->armv7m), &error_fatal);


    // Adding in devices for STM32H753ZI - there are loads
    create_unimplemented_device("TIM2",        0x40000000, 0x00000400);
    create_unimplemented_device("TIM3",        0x40000400, 0x00000400);
    create_unimplemented_device("TIM4",        0x40000800, 0x00000400);
    create_unimplemented_device("TIM5",        0x40000C00, 0x00000400);
    create_unimplemented_device("TIM6",        0x40001000, 0x00000400);
    create_unimplemented_device("TIM7",        0x40001400, 0x00000400);
    create_unimplemented_device("TIM12",       0x40001800, 0x00000400);
    create_unimplemented_device("TIM13",       0x40001C00, 0x00000400);
    create_unimplemented_device("TIM14",       0x40002000, 0x00000400);
    create_unimplemented_device("LPTIM1",      0x40002400, 0x00000400);
    create_unimplemented_device("RESERVED1",   0x40002800, 0x00001000);
    create_unimplemented_device("SPI2",        0x40003800, 0x00000400);
    create_unimplemented_device("SPI3",        0x40003C00, 0x00000400);
    create_unimplemented_device("SPDIFRX1",    0x40004000, 0x00000400);
    // PRM NOTE: USARTS are created as smulated devices lower down in this code.
    //create_unimplemented_device("USART2",      0x40004400, 0x00000400);
    //create_unimplemented_device("USART3",      0x40004800, 0x00000400);
    //create_unimplemented_device("USART4",      0x40004C00, 0x00000400);
    //create_unimplemented_device("USART5",      0x40005000, 0x00000400);
    create_unimplemented_device("I2C1",        0x40005400, 0x00000400);
    create_unimplemented_device("I2C2",        0x40005800, 0x00000400);
    create_unimplemented_device("I2C3",        0x40005C00, 0x00000400);
    create_unimplemented_device("RESERVED2",   0x40006000, 0x00000C00);
    create_unimplemented_device("HDMI-CEC",    0x40006C00, 0x00000400);
    create_unimplemented_device("RESERVED3",   0x40007000, 0x00000400);
    create_unimplemented_device("DAC1",        0x40007400, 0x00000400);
    create_unimplemented_device("UART7",       0x40007800, 0x00000400);
    create_unimplemented_device("UART8",       0x40007C00, 0x00000400);
    create_unimplemented_device("RESERVED4",   0x40008000, 0x00000400);
    create_unimplemented_device("CRS",         0x40008400, 0x00000400);
    create_unimplemented_device("SWPMI",       0x40008800, 0x00000400);
    create_unimplemented_device("RESERVED5",   0x40008C00, 0x00000400);
    create_unimplemented_device("OPAMP",       0x40009000, 0x00000400);
    create_unimplemented_device("MDIOS",       0x40009400, 0x00000400);
    create_unimplemented_device("FDCAN1",      0x4000A000, 0x00000400);
    create_unimplemented_device("FDCAN2",      0x4000A400, 0x00000400);
    create_unimplemented_device("CAN CCU",     0x4000A800, 0x00000400);
    create_unimplemented_device("CAN Msg RAM", 0x4000AC00, 0x00002800);
    create_unimplemented_device("RESERVED6",   0x4000D400, 0x00002C00);
    create_unimplemented_device("TIM1",        0x40010000, 0x00000400);
    create_unimplemented_device("TIM8",        0x40010400, 0x00000400);
    create_unimplemented_device("RESERVED7",   0x40010800, 0x00000800);
    // PRM NOTE: USARTS are created as smulated devices lower down in this code.
    //create_unimplemented_device("USART1",      0x40011000, 0x00000400);
    //create_unimplemented_device("USART6",      0x40011400, 0x00000400);
    create_unimplemented_device("RESERVED8",   0x40011800, 0x00001800);
    create_unimplemented_device("SPI1 12S1",   0x40013000, 0x00000400);
    create_unimplemented_device("SPI4",        0x40013400, 0x00000400);
    create_unimplemented_device("RESERVED9",   0x40013800, 0x00000800); 
    create_unimplemented_device("TIM15",       0x40014000, 0x00000400);
    create_unimplemented_device("TIM16",       0x40014400, 0x00000400);
    create_unimplemented_device("TIM17",       0x40014800, 0x00000400);
    create_unimplemented_device("RESERVED10",  0x40014C00, 0x00000400);
    create_unimplemented_device("SPI5",        0x40015000, 0x00000400);
    create_unimplemented_device("RESERVED10",  0x40015400, 0x00000400);
    create_unimplemented_device("SAI1",        0x40015800, 0x00000400);
    create_unimplemented_device("SAI2",        0x40015C00, 0x00000400);
    create_unimplemented_device("SAI3",        0x40016000, 0x00000400);
    create_unimplemented_device("RESERVED11",  0x40016400, 0x00000C00);
    create_unimplemented_device("DFSDM1",      0x40017000, 0x00000400);
    create_unimplemented_device("HRTIM",       0x40017400, 0x00000400);
    create_unimplemented_device("RESERVED12",  0x40017800, 0x00007800);
    create_unimplemented_device("DMA1",        0x40020000, 0x00000400);
    create_unimplemented_device("DMA2",        0x40020400, 0x00000400);
    create_unimplemented_device("DMAMUX1",     0x40020800, 0x00000400);
    create_unimplemented_device("RESERVED13",  0x40020C00, 0x00002400);
    create_unimplemented_device("ADC1-ADC2",   0x40022000, 0x00000400);
    create_unimplemented_device("RESERVED14",  0x40022400, 0x00005C00);
    create_unimplemented_device("ETHERNET MAC",0x40028000, 0x00001400);
    create_unimplemented_device("RESERVED15",  0x40029400, 0x0001C600);
    create_unimplemented_device("USB1 OTG_HS", 0x40040000, 0x00040000);
    create_unimplemented_device("USB2 OTG_FS", 0x40080000, 0x00040000);
    create_unimplemented_device("RESERVED16",  0x400C0000, 0x07F60000);
    create_unimplemented_device("DCMI",        0x48020000, 0x00000400);
    create_unimplemented_device("RESERVED17",  0x48020400, 0x00000C00);
    create_unimplemented_device("CRYPTO",      0x48021000, 0x00000400);
    create_unimplemented_device("HASH",        0x48021400, 0x00000400);
    create_unimplemented_device("RNG",         0x48021800, 0x00000400);
    create_unimplemented_device("RESERVED18",  0x48021C00, 0x00000800);
    create_unimplemented_device("SDMMC2",      0x48022400, 0x00000400);
    create_unimplemented_device("SDMMC2 DELAY",0x48022800, 0x00000400);
    create_unimplemented_device("RESERVED19",  0x48022C00, 0x00000400);
    create_unimplemented_device("RAMECC2",     0x48023000, 0x00000400);
    create_unimplemented_device("RESERVED20",  0x48023000, 0x07FDDC00);
    create_unimplemented_device("LTDC",        0x50001000, 0x00001000);
    create_unimplemented_device("RESERVED21",  0x50002000, 0x00001000);
    create_unimplemented_device("WWDG1",       0x50003000, 0x00001000);
    create_unimplemented_device("RESERVED21",  0x50004000, 0x00FFC000);
    create_unimplemented_device("GPV",         0x51000000, 0x00100000);
    create_unimplemented_device("MDMA",        0x52000000, 0x00001000);
    create_unimplemented_device("DMA2D",       0x52001000, 0x00001000);
    create_unimplemented_device("Flash reg",   0x52002000, 0x00001000);
    create_unimplemented_device("JPEG",        0x52003000, 0x00001000);
    create_unimplemented_device("FMC",         0x52004000, 0x00001000);
    create_unimplemented_device("QUADSPI",     0x52005000, 0x00001000);
    create_unimplemented_device("QUADSPI DLY", 0x52006000, 0x00001000);
    create_unimplemented_device("SDMMC1",      0x52007000, 0x00001000);
    create_unimplemented_device("SDMMC1 DLY",  0x52008000, 0x00001000);
    create_unimplemented_device("RAMECC1",     0x52009000, 0x00000400);
    create_unimplemented_device("RESERVED22",  0x52009400, 0x05FF6C00);
    create_unimplemented_device("EXTI",        0x58000000, 0x00000400);
    create_unimplemented_device("SYSCFG",      0x58000400, 0x00000400);
    create_unimplemented_device("LPUART1",     0x58000C00, 0x00000400);
    create_unimplemented_device("RESERVED23",  0x58001000, 0x00000400);
    create_unimplemented_device("SPI6",        0x58001400, 0x00000400);
    create_unimplemented_device("RESERVED24",  0x58001800, 0x00000400);
    create_unimplemented_device("I2C4",        0x58001C00, 0x00000400);
    create_unimplemented_device("RESERVED25",  0x58002000, 0x00000400);
    create_unimplemented_device("LPTIM2",      0x58002400, 0x00000400);
    create_unimplemented_device("LPTIM3",      0x58002800, 0x00000400);
    create_unimplemented_device("LPTIM4",      0x58002C00, 0x00000400);
    create_unimplemented_device("LPTIM5",      0x58003000, 0x00000400);
    create_unimplemented_device("RESERVED26",  0x58003400, 0x00000400);
    create_unimplemented_device("COMP1-COMP2", 0x58003800, 0x00000400);
    create_unimplemented_device("VREF",        0x58003C00, 0x00000400);
    create_unimplemented_device("RTC & BKP",   0x58004000, 0x00000400);
    create_unimplemented_device("RESERVED27",  0x58004400, 0x00000400);
    create_unimplemented_device("IWDG1",       0x58004800, 0x00000400);
    create_unimplemented_device("RESERVED28",  0x58004C00, 0x00000800);
    create_unimplemented_device("SAI4",        0x58005400, 0x00000400);
    create_unimplemented_device("RESERVED29",  0x58005800, 0x0001A800);
    create_unimplemented_device("GPIOA",       0x58020000, 0x00000400);
    create_unimplemented_device("GPIOB",       0x58020400, 0x00000400);
    create_unimplemented_device("GPIOC",       0x58020800, 0x00000400);
    create_unimplemented_device("GPIOD",       0x58020C00, 0x00000400);
    create_unimplemented_device("GPIOE",       0x58021000, 0x00000400);
    create_unimplemented_device("GPIOF",       0x58021400, 0x00000400);
    create_unimplemented_device("GPIOG",       0x58021800, 0x00000400);
    create_unimplemented_device("GPIOH",       0x58021C00, 0x00000400);
    create_unimplemented_device("GPIOI",       0x58022000, 0x00000400);
    create_unimplemented_device("GPIOJ",       0x58022400, 0x00000400);
    create_unimplemented_device("GPIOK",       0x58022800, 0x00000400);
    create_unimplemented_device("RESERVED30",  0x58022C00, 0x00001800);
    // PRM NOTE: added emulation support below
    //create_unimplemented_device("RCC",         0x58024400, 0x00000400);
    // PRM NOTE: emulated below
    //create_unimplemented_device("PWR",         0x58024800, 0x00000400);
    create_unimplemented_device("CRC",         0x58024C00, 0x00000400);
    create_unimplemented_device("RESERVED31",  0x58025000, 0x00000400);
    create_unimplemented_device("BDMA",        0x58025400, 0x00000400);
    create_unimplemented_device("DMA MUX2",    0x58025800, 0x00000400);
    create_unimplemented_device("RESERVED32",  0x58025C00, 0x00000400);
    create_unimplemented_device("ADC3",        0x58026000, 0x00000400);
    create_unimplemented_device("HSEM",        0x58026400, 0x00000400);
    create_unimplemented_device("RAM ECC 3",   0x58027000, 0x00000400);
    create_unimplemented_device("DBG",         0x5C001000, 0x00000400);
    
    qemu_log("Mapped unimplemented STM32 Devices\n");
    
    // PRM initialise the PWR peripheral.
    object_initialize_child(OBJECT(mms), "pwr", &mms->pwr, TYPE_STM32F7XX_PWR);

    if (!sysbus_realize(SYS_BUS_DEVICE(&mms->pwr), &error_fatal))
    {
        qemu_log("Can't initialise the PWR hardware\n");
        return;
    }
    pwr = DEVICE(&(mms->pwr));

    qemu_log("pwr device is 0x%lx\n", (long unsigned int)pwr);
    
    busdev = SYS_BUS_DEVICE(pwr);

    qemu_log("busdev device is 0x%lx\n", (long unsigned int)busdev);
    
    // map the io to physical addresses
    sysbus_mmio_map(busdev, 0, 0x58024800);

    qemu_log("mapped pwr io mem space");


    
    // PRM initialise the RCC peripheral
    object_initialize_child(OBJECT(mms), "rcc", &mms->rcc, TYPE_STM32F7XX_RCC);
    
    if (!sysbus_realize(SYS_BUS_DEVICE(&mms->rcc), &error_fatal))
    {
        qemu_log("Can't initialise the RCC hardware\n");
        return;
    }
    rcc = DEVICE(&(mms->rcc));

    qemu_log("rcc device is 0x%lx\n", (long unsigned int)rcc);
    
    busdev = SYS_BUS_DEVICE(rcc);

    qemu_log("busdev device is 0x%lx\n", (long unsigned int)busdev);
    
    // map the io to physical addresses
    sysbus_mmio_map(busdev, 0, 0x58024400);

    qemu_log("mapped pwr io mem space");
    
    
    switch (mmc->fpga_type) {
    case FPGA_AN385:
    case FPGA_AN386:
    case FPGA_AN500:
    {
        for (i = 0; i < STM_NUM_USARTS; i++)
        {
            static const uint32_t usart_addr[STM_NUM_USARTS] = {0x40011000, 0x40004400,
                                              0x40004800, 0x40004C00,
                                              0x40005000, 0x40011400};
            
            // initialise the child object for the USARTS
            object_initialize_child(OBJECT(mms), "usart[*]", &mms->usart[i], TYPE_STM32F2XX_USART);

            // get a handle for the device
            usart = DEVICE(&(mms->usart[i]));

            // PRM NOTE: only serial device 0 is actually connected to anything.
            // At the moment, I am unsure of how to connect other serial devices.
            // I think it is possible through the QEMU command line but I am not sure how?
            // On the Nucleo STM32H753ZI board, USART3 (offset 2 into usart_addr) is the
            // one that connects through STLink to a PC UART.
            if (i==2)
            {
                qdev_prop_set_chr(usart, "chardev", serial_hd(0));
            }
            if (!sysbus_realize(SYS_BUS_DEVICE(&mms->usart[i]), &error_fatal))
            {
                qemu_log("Can't initialise the USART number %d\n", i+1 );
                return;
            }
            busdev = SYS_BUS_DEVICE(usart);

            // map the io to physical addresses
            sysbus_mmio_map(busdev, 0,usart_addr[i]);
        }
        break;
    }
    case FPGA_AN511:
    {
        /* The overflow IRQs for all UARTs are ORed together.
         * Tx and Rx IRQs for each UART are ORed together.
         */
        Object *orgate;
        DeviceState *orgate_dev;

        orgate = object_new(TYPE_OR_IRQ);
        object_property_set_int(orgate, "num-lines", 10, &error_fatal);
        qdev_realize(DEVICE(orgate), NULL, &error_fatal);
        orgate_dev = DEVICE(orgate);
        qdev_connect_gpio_out(orgate_dev, 0, qdev_get_gpio_in(armv7m, 12));
        break;
    }
    default:
        g_assert_not_reached();
    }
    for (i = 0; i < 4; i++) {
        static const hwaddr gpiobase[] = {0x40010000, 0x40011000,
                                          0x40012000, 0x40013000};
        create_unimplemented_device("cmsdk-ahb-gpio", gpiobase[i], 0x1000);
    }

    /* CMSDK APB subsystem */
    cmsdk_apb_timer_create(0x40000000, qdev_get_gpio_in(armv7m, 8), SYSCLK_FRQ);
    cmsdk_apb_timer_create(0x40001000, qdev_get_gpio_in(armv7m, 9), SYSCLK_FRQ);
    object_initialize_child(OBJECT(mms), "dualtimer", &mms->dualtimer,
                            TYPE_CMSDK_APB_DUALTIMER);
    qdev_prop_set_uint32(DEVICE(&mms->dualtimer), "pclk-frq", SYSCLK_FRQ);
    sysbus_realize(SYS_BUS_DEVICE(&mms->dualtimer), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(&mms->dualtimer), 0,
                       qdev_get_gpio_in(armv7m, 10));
    sysbus_mmio_map(SYS_BUS_DEVICE(&mms->dualtimer), 0, 0x40002000);
    object_initialize_child(OBJECT(mms), "watchdog", &mms->watchdog,
                            TYPE_CMSDK_APB_WATCHDOG);
    qdev_prop_set_uint32(DEVICE(&mms->watchdog), "wdogclk-frq", SYSCLK_FRQ);
    sysbus_realize(SYS_BUS_DEVICE(&mms->watchdog), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(&mms->watchdog), 0,
                       qdev_get_gpio_in_named(armv7m, "NMI", 0));
    sysbus_mmio_map(SYS_BUS_DEVICE(&mms->watchdog), 0, 0x40008000);

    /* FPGA APB subsystem */
    object_initialize_child(OBJECT(mms), "scc", &mms->scc, TYPE_MPS2_SCC);
    sccdev = DEVICE(&mms->scc);
    qdev_prop_set_uint32(sccdev, "scc-cfg4", 0x2);
    qdev_prop_set_uint32(sccdev, "scc-aid", 0x00200008);
    qdev_prop_set_uint32(sccdev, "scc-id", mmc->scc_id);
    sysbus_realize(SYS_BUS_DEVICE(&mms->scc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(sccdev), 0, 0x4002f000);
    object_initialize_child(OBJECT(mms), "fpgaio",
                            &mms->fpgaio, TYPE_MPS2_FPGAIO);
    qdev_prop_set_uint32(DEVICE(&mms->fpgaio), "prescale-clk", 25000000);
    sysbus_realize(SYS_BUS_DEVICE(&mms->fpgaio), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&mms->fpgaio), 0, 0x40028000);
    sysbus_create_simple(TYPE_PL022, 0x40025000,        /* External ADC */
                         qdev_get_gpio_in(armv7m, 22));
    for (i = 0; i < 2; i++) {
        static const int spi_irqno[] = {11, 24};
        static const hwaddr spibase[] = {0x40020000,    /* APB */
                                         0x40021000,    /* LCD */
                                         0x40026000,    /* Shield0 */
                                         0x40027000};   /* Shield1 */
        DeviceState *orgate_dev;
        Object *orgate;
        int j;

        orgate = object_new(TYPE_OR_IRQ);
        object_property_set_int(orgate, "num-lines", 2, &error_fatal);
        orgate_dev = DEVICE(orgate);
        qdev_realize(orgate_dev, NULL, &error_fatal);
        qdev_connect_gpio_out(orgate_dev, 0,
                              qdev_get_gpio_in(armv7m, spi_irqno[i]));
        for (j = 0; j < 2; j++) {
            sysbus_create_simple(TYPE_PL022, spibase[2 * i + j],
                                 qdev_get_gpio_in(orgate_dev, j));
        }
    }
    for (i = 0; i < 4; i++) {
        static const hwaddr i2cbase[] = {0x40022000,    /* Touch */
                                         0x40023000,    /* Audio */
                                         0x40029000,    /* Shield0 */
                                         0x4002a000};   /* Shield1 */
        sysbus_create_simple(TYPE_ARM_SBCON_I2C, i2cbase[i], NULL);
    }
    create_unimplemented_device("i2s", 0x40024000, 0x400);

    /* In hardware this is a LAN9220; the LAN9118 is software compatible
     * except that it doesn't support the checksum-offload feature.
     */
    lan9118_init(&nd_table[0], mmc->ethernet_base,
                 qdev_get_gpio_in(armv7m,
                                  mmc->fpga_type == FPGA_AN511 ? 47 : 13));

    system_clock_scale = NANOSECONDS_PER_SECOND / SYSCLK_FRQ;

    armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename,
                       0x400000);
}

static void mps2_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = mps2_common_init;
    mc->max_cpus = 1;
    mc->default_ram_size = 16 * MiB;
    mc->default_ram_id = "mps.ram";
}

static void mps2_an385_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    MPS2MachineClass *mmc = MPS2_MACHINE_CLASS(oc);

    mc->desc = "ARM MPS2 with AN385 FPGA image for Cortex-M3";
    mmc->fpga_type = FPGA_AN385;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m3");
    mmc->scc_id = 0x41043850;
    mmc->psram_base = 0x21000000;
    mmc->ethernet_base = 0x40200000;
    mmc->has_block_ram = true;
}

static void mps2_an386_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    MPS2MachineClass *mmc = MPS2_MACHINE_CLASS(oc);

    mc->desc = "ARM MPS2 with AN386 FPGA image for Cortex-M4";
    mmc->fpga_type = FPGA_AN386;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m4");
    mmc->scc_id = 0x41043860;
    mmc->psram_base = 0x21000000;
    mmc->ethernet_base = 0x40200000;
    mmc->has_block_ram = true;
}

static void mps2_an500_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    MPS2MachineClass *mmc = MPS2_MACHINE_CLASS(oc);

    mc->desc = "STM32H753ZI Nucleo Cortex-M7";
    mmc->fpga_type = FPGA_AN500;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m7");
    mmc->scc_id = 0x41045000;
    mmc->psram_base = 0x60000000;
    mmc->ethernet_base = 0xa0000000;
    mmc->has_block_ram = false;
}

static void mps2_an511_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    MPS2MachineClass *mmc = MPS2_MACHINE_CLASS(oc);

    mc->desc = "ARM MPS2 with AN511 DesignStart FPGA image for Cortex-M3";
    mmc->fpga_type = FPGA_AN511;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m3");
    mmc->scc_id = 0x41045110;
    mmc->psram_base = 0x21000000;
    mmc->ethernet_base = 0x40200000;
    mmc->has_block_ram = false;
}

static const TypeInfo mps2_info = {
    .name = TYPE_MPS2_MACHINE,
    .parent = TYPE_MACHINE,
    .abstract = true,
    .instance_size = sizeof(MPS2MachineState),
    .class_size = sizeof(MPS2MachineClass),
    .class_init = mps2_class_init,
};

static const TypeInfo mps2_an385_info = {
    .name = TYPE_MPS2_AN385_MACHINE,
    .parent = TYPE_MPS2_MACHINE,
    .class_init = mps2_an385_class_init,
};

static const TypeInfo mps2_an386_info = {
    .name = TYPE_MPS2_AN386_MACHINE,
    .parent = TYPE_MPS2_MACHINE,
    .class_init = mps2_an386_class_init,
};

static const TypeInfo mps2_an500_info = {
    .name = TYPE_MPS2_AN500_MACHINE,
    .parent = TYPE_MPS2_MACHINE,
    .class_init = mps2_an500_class_init,
};

static const TypeInfo mps2_an511_info = {
    .name = TYPE_MPS2_AN511_MACHINE,
    .parent = TYPE_MPS2_MACHINE,
    .class_init = mps2_an511_class_init,
};

static void mps2_machine_init(void)
{
    type_register_static(&mps2_info);
    type_register_static(&mps2_an385_info);
    type_register_static(&mps2_an386_info);
    type_register_static(&mps2_an500_info);
    type_register_static(&mps2_an511_info);
}

type_init(mps2_machine_init);
