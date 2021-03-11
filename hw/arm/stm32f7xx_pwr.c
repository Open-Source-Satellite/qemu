/*-
 * Copyright (c) 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
 * QEMU stm32f7xx RTC emulation
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "hw/arm/stm32f7xx_pwr.h"

//#define DEBUG_STM32F7XX_PWR
#ifdef DEBUG_STM32F7XX_PWR
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_STM32F7XX_PWR: " fmt , ## __VA_ARGS__); \
     \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

static uint64_t
f7xx_pwr_read(void *arg, hwaddr addr, unsigned int size)
{
    STM32F7XXPwrState *s = arg;
    uint32_t r;
    int offset = addr & 0x3;
    uint32_t value;

    qemu_log("PWR: reading register 0x%" HWADDR_PRIx "\n", addr);

    addr >>= 2; // shift down the address 
    // if this is request for the D3CR
    if (addr == R_PWR_D3CR)
    {
        // return VOSRDY if the clock is enabled
        if (0 != (s->regs[addr] & 0xC000))
        {
            qemu_log("PWR: Setting VOSRDY");
            r = 0x6000; // set VOSRDY
        }
        else
        {
            r = s->regs[addr];
        }
    }
    else // else, just return the value in the register
    {
        value = s->regs[addr];
        r = (value >> offset * 8) & ((1ull << (8 * size)) - 1); 
    }

    qemu_log("PWR: got data 0x%x\n\n", r);
    return r;
}
static void
f7xx_pwr_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
    STM32F7XXPwrState *s = arg;
    int offset = addr & 0x3;

    qemu_log("PWR: writing address 0x%" HWADDR_PRIx "\n", addr);
    
    addr >>= 2;
    if (addr >= R_PWR_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "invalid write f7xx pwr register 0x%x\n",
          (unsigned int)addr << 2);
        return;
    }

    switch(size) {
    case 1:
        data = (s->regs[addr] & ~(0xff << (offset * 8))) | data << (offset * 8);
        break;
    case 2:
        data = (s->regs[addr] & ~(0xffff << (offset * 8))) | data << (offset * 8);
        break;
    case 4:
        break;
    default:
        abort();
    }

    switch(addr) {
    case R_PWR_CR:
    case R_PWR_CSR:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "f7xx pwr unimplemented write 0x%x+%u size %u val 0x%x\n",
          (unsigned int)addr << 2, offset, size, (unsigned int)data);
    }

    // Save new value
    s->regs[addr] = data;
}

static const MemoryRegionOps f7xx_pwr_ops = {
    .read = f7xx_pwr_read,
    .write = f7xx_pwr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void f7xx_pwr_reset(DeviceState *dev)
{
    STM32F7XXPwrState *s = STM32F7XX_PWR(dev);
    //DO_UPCAST(STM32F7XXPwrState, busdev, SYS_BUS_DEVICE(dev));

    memset(s->regs, 0, sizeof(s->regs));
}


static int
f7xx_pwr_init(SysBusDevice *dev)
{
    STM32F7XXPwrState *s = STM32F7XX_PWR(dev);
    //STM32F7XXPwrState *s = DO_UPCAST(STM32F7XXPwrState, busdev, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &f7xx_pwr_ops, s, "pwr", 0x3FF);
    sysbus_init_mmio(dev, &s->iomem);

    s->regs[R_PWR_CR] = 0;
    s->regs[R_PWR_CSR] = 0;
    s->regs[R_PWR_D3CR] = 0x2000; // Set the VOSRDY bit

    return 0;
}

static Property f7xx_pwr_properties[] = {
    {},
};

static void
f7xx_pwr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    //SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    // PRM I think the sc->init is deprecated and replaced with realize call back
    //sc->init = f7xx_pwr_init;
    dc->realize = (void*)f7xx_pwr_init;
    dc->reset = f7xx_pwr_reset;
    device_class_set_props(dc, f7xx_pwr_properties);
}

static const TypeInfo
f7xx_pwr_info = {
    .name          = TYPE_STM32F7XX_PWR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F7XXPwrState),
    .class_init    = f7xx_pwr_class_init,
};

static void
f7xx_pwr_register_types(void)
{
    type_register_static(&f7xx_pwr_info);
}

type_init(f7xx_pwr_register_types)
