/***********************************************************************************
 *  @file stm32h753zi.c
 ***********************************************************************************
 *   _  _____ ____  ____  _____ 
 *  | |/ /_ _/ ___||  _ \| ____|
 *  | ' / | |\___ \| |_) |  _|  
 *  | . \ | | ___) |  __/| |___ 
 *  |_|\_\___|____/|_|   |_____|
 *
 ***********************************************************************************
 *  Copyright (c) 2021 KISPE Space Systems Ltd.
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ***********************************************************************************
 *  This code has been written to emulate the STM32H753ZI chip as running on the
 *  STM32H753ZI Nucleo board. The emulation is currently basic, including USART, RCC &
 *  PWR peripherals.
 *  Code can be built under the STM32Cube IDE (as can be used for the Nucleo board) and
 *  then modified slightly in order to run under this QEMU emulation of the board and
 *  System On a Chip processor. USART3 is attached to QEMU stdio in order to emulate
 *  the ST Link UART output that can be seen on the real hardware. This code is based
 *  upon the MPS2-AN500 code written by Peter Maydell with STM peripherals that are 
 *  sourced from the XPack qemu fork that specialises in the emulation of STM micros.
 *  It is forked from QEMU 5.2 where Cortex M7 emulation was added to QEMU.        
 *  @author: Paul Madle                     
 ***********************************************************************************/

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
#include "net/net.h"
#include "qom/object.h"
#include "hw/char/stm32f2xx_usart.h"
#include "hw/arm/stm32f7xx_pwr.h"
#include "hw/arm/stm32f7xx_rcc.h"

#define STM_NUM_USARTS 6

struct STM32H753MachineClass {
    MachineClass parent;
};

struct STM32H753MachineState {
    MachineState parent;

    ARMv7MState armv7m;
    MemoryRegion ssram1;
    MemoryRegion ssram1_m;
    MemoryRegion ssram23;
    MemoryRegion ssram23_m;
    MemoryRegion sram;
    
    /* emulated STM32 peripheral hardware */
    STM32F2XXUsartState usart[STM_NUM_USARTS];
    STM32F7XXPwrState   pwr;
    Stm32f7xxRcc        rcc;
};

#define TYPE_STM32H753_MACHINE "stm32h753"
#define TYPE_STM32H753_MACHINE_NAME MACHINE_TYPE_NAME("stm32h753-nucleo")

OBJECT_DECLARE_TYPE(STM32H753MachineState, STM32H753MachineClass, STM32H753_MACHINE)

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

static void stm32h753_common_init(MachineState *machine)
{
    STM32H753MachineState *mms = STM32H753_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    DeviceState *armv7m, *usart, *pwr, *rcc;
    SysBusDevice *busdev;
    int i = 0;
    
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

    /*
     * Initialise the memory: TODO: these addresses are for the MPS2-AN500 chip.
     * It is not urgent, but it would be good to switch these addresses so that
     * QEMU is emulating the STM32 Flash memory for code execution. At the moment,
     * I am changing the linker file for the Nucleo code so that it targets the MPS2
     * memory space. The MPS2-AN500 memory space looks like this:
     * 
     * 0x00000000 .. 0x003fffff : ZBT SSRAM1
     * 0x00400000 .. 0x007fffff : mirror of ZBT SSRAM1
     * 0x20000000 .. 0x203fffff : ZBT SSRAM 2&3
     * 0x20400000 .. 0x207fffff : mirror of ZBT SSRAM 2&3
     */
    make_ram(&mms->ssram1, "mps.ssram1", 0x0, 0x400000);
    make_ram_alias(&mms->ssram1_m, "mps.ssram1_m", &mms->ssram1, 0x400000);
    make_ram(&mms->ssram23, "mps.ssram23", 0x20000000, 0x400000);
    make_ram_alias(&mms->ssram23_m, "mps.ssram23_m",
                  &mms->ssram23, 0x20400000);

    // Initialise the Cortex M7 core.
    object_initialize_child(OBJECT(mms), "armv7m", &mms->armv7m, TYPE_ARMV7M);
    
    armv7m = DEVICE(&mms->armv7m);

    // TODO: this is a hangup from the MPS2-AN500, it is surely wrong for the STM32H753ZI
    qdev_prop_set_uint32(armv7m, "num-irq", 32);

    // Get the Cortex-M7 core going.
    qdev_prop_set_string(armv7m, "cpu-type", machine->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&mms->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&mms->armv7m), &error_fatal);


    /* 
     * These are all of the peripherals on the STM32H7 that are (as yet) unimplemented.
     * In order to get code enough to emulate FreeRTOS correctly, I think we would need:
     * timers
     * systick (at least get the interrupt stuff working well)
     * interrupts
     *
     * It would also be nice to emulate GPIO too (maybe with a graphical display that shows LEDs etc?).
     */
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
    
    // Initialise the STM32 PWR peripheral.
    object_initialize_child(OBJECT(mms), "pwr", &mms->pwr, TYPE_STM32F7XX_PWR);

    if (!sysbus_realize(SYS_BUS_DEVICE(&mms->pwr), &error_fatal))
    {
        qemu_log("Can't initialise the PWR hardware\n");
        return;
    }
    pwr = DEVICE(&(mms->pwr));
    
    busdev = SYS_BUS_DEVICE(pwr);

    // map the io to physical addresses
    sysbus_mmio_map(busdev, 0, 0x58024800);


    
    // initialise the RCC peripheral
    object_initialize_child(OBJECT(mms), "rcc", &mms->rcc, TYPE_STM32F7XX_RCC);
    
    if (!sysbus_realize(SYS_BUS_DEVICE(&mms->rcc), &error_fatal))
    {
        qemu_log("Can't initialise the RCC hardware\n");
        return;
    }
    rcc = DEVICE(&(mms->rcc));

    busdev = SYS_BUS_DEVICE(rcc);

    // map the io to physical addresses
    sysbus_mmio_map(busdev, 0, 0x58024400);
    

    // USART initialisation
    static const uint32_t usart_addr[STM_NUM_USARTS] = {0x40011000, 0x40004400,
                                                        0x40004800, 0x40004C00,
                                                        0x40005000, 0x40011400};
     // Loop around,  initialising the UARTS
    for (i=0 ; i<STM_NUM_USARTS ; i++)
    {       
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

    system_clock_scale = NANOSECONDS_PER_SECOND / SYSCLK_FRQ;

    armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename,
                       0x400000);
}

static void stm32h753_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = stm32h753_common_init;
    mc->max_cpus = 1;
    mc->default_ram_size = 16 * MiB;
    mc->default_ram_id = "mps.ram";
}


static void stm32h753_nucleo_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "STM32H753ZI Nucleo Cortex-M7";
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m7");
}


static const TypeInfo stm32h753_info = {
    .name = TYPE_STM32H753_MACHINE,
    .parent = TYPE_MACHINE,
    .abstract = true,
    .instance_size = sizeof(STM32H753MachineState),
    .class_size = sizeof(STM32H753MachineClass),
    .class_init = stm32h753_class_init,
};

static const TypeInfo stm32h753_nucleo_info = {
    .name = TYPE_STM32H753_MACHINE_NAME,
    .parent = TYPE_STM32H753_MACHINE,
    .class_init = stm32h753_nucleo_class_init,
};



static void stm32h753_machine_init(void)
{
    type_register_static(&stm32h753_info);
    type_register_static(&stm32h753_nucleo_info);
}

type_init(stm32h753_machine_init);
