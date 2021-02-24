/***********************************************************************************
 *  @file stm32f7xx_pwr.h
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
 *  Header file, the interface for the STM32F7 PWR peripheral       
 *  @author: Paul Madle                     
 ***********************************************************************************/

/*
 * QEMU stm32f7xx PWR emulation
 */

#ifndef __STM32F7XX_PWR__
#define __STM32F7XX_PWR__

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qom/object.h"

#define R_PWR_CR      (0x00/4)
#define R_PWR_CR_LPDS   0x00000001
#define R_PWR_CR_PDDS   0x00000002

#define R_PWR_CSR     (0x04/4)

#define R_PWR_MAX     (0x1C/4)

#define R_PWR_D3CR    (0x18/4)

#define TYPE_STM32F7XX_PWR "f7xx_pwr"

// create the object
OBJECT_DECLARE_SIMPLE_TYPE(STM32F7XXPwrState, STM32F7XX_PWR)
    
typedef struct STM32F7XXPwrState {
    SysBusDevice  busdev;
    MemoryRegion  iomem;

    uint32_t      regs[R_PWR_MAX];
} STM32F7XXPwrState;

#endif /* __STM32F7XX_PWR__ */
