/* This file is the part of the Lightweight USB device Stack for STM32 microcontrollers
 *
 * Copyright ©2016 Dmitry Filimonchuk <dmitrystu[at]gmail[dot]com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//
// Modified 27 Aug 2018 by Mark Haun for STM32L496 support (same as STM32L476)
// and to move dmitrystu's stm32.h macros here.
//

#ifndef _USB_H_
#define _USB_H_
#if defined(__cplusplus)
    extern "C" {
#endif

#include "stm32l4xx.h"   // mh: get "STM32L496xx" defined

// mh: Bit-operation macros from stm32.h (see https://github.com/dmitrystu/stm32h)
/* modify bitfield */
#define _BMD(reg, msk, val)     (reg) = (((reg) & ~(msk)) | (val))
/* set bitfield */
#define _BST(reg, bits)         (reg) = ((reg) | (bits))
/* clear bitfield */
#define _BCL(reg, bits)         (reg) = ((reg) & ~(bits))
/* wait until bitfield set */
#define _WBS(reg, bits)         while(((reg) & (bits)) == 0)
/* wait until bitfield clear */
#define _WBC(reg, bits)         while(((reg) & (bits)) != 0)
/* wait for bitfield value */
#define _WVL(reg, msk, val)     while(((reg) & (msk)) != (val))
/* bit value */
#define _BV(bit)                (0x01 << (bit))

#include "usbd_core.h"
#if !defined(__ASSEMBLER__)
#include "usb_std.h"
#endif

#if defined(STM32L052xx) || defined(STM32L053xx) || \
    defined(STM32L062xx) || defined(STM32L063xx) || \
    defined(STM32L072xx) || defined(STM32L073xx) || \
    defined(STM32L082xx) || defined(STM32L083xx) || \
    defined(STM32L432xx) || defined(STM32L433xx) || \
    defined(STM32L442xx) || defined(STM32L443xx) || \
    defined(STM32L452xx) || defined(STM32L462xx) || \
    defined(STM32F042x6) || defined(STM32F048xx) || \
    defined(STM32F070x6) || defined(STM32F070xB) || \
    defined(STM32F072xB) || defined(STM32F078xx)

    #define USBD_STM32L052

    #if !defined(__ASSEMBLER__)
    extern const struct usbd_driver usbd_devfs;
    extern const struct usbd_driver usbd_devfs_asm;
    #if defined(USBD_ASM_DRIVER)
    #define usbd_hw usbd_devfs_asm
    #else
    #define usbd_hw usbd_devfs
    #endif
    #endif

#elif defined(STM32L1)

    #define USBD_STM32L100

    #if !defined(__ASSEMBLER__)
    extern const struct usbd_driver usbd_devfs;
    extern const struct usbd_driver usbd_devfs_asm;
    #if defined(USBD_ASM_DRIVER)
    #define usbd_hw usbd_devfs_asm
    #else
    #define usbd_hw usbd_devfs
    #endif
    #endif

#elif defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L496xx)  // added mh 27aug18

    #define USBD_STM32L476

    #if !defined(__ASSEMBLER__)
    extern const struct usbd_driver usbd_otgfs;
    #define usbd_hw usbd_otgfs
    #endif

#elif defined(STM32F405xx) || defined(STM32F415xx) || \
      defined(STM32F407xx) || defined(STM32F417xx) || \
      defined(STM32F427xx) || defined(STM32F437xx) || \
      defined(STM32F429xx) || defined(STM32F439xx)

    #define USBD_STM32F429

    #if !defined(__ASSEMBLER__)
    extern const struct usbd_driver usbd_otgfs;
    #define usbd_hw usbd_otgfs
    #endif

#elif defined(STM32F102x6) || defined(STM32F102xB) || \
      defined(STM32F103x6) || defined(STM32F103xB) || \
      defined(STM32F103xE) || defined(STM32F103xG) || \
      defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) || \
      defined(STM32F303xC) || defined(STM32F303xE) || \
      defined(STM32F373xC)

    #define USBD_STM32F103

    #if !defined(__ASSEMBLER__)
    extern const struct usbd_driver usbd_devfs;
    extern const struct usbd_driver usbd_devfs_asm;
    #if defined(USBD_ASM_DRIVER)
    #define usbd_hw usbd_devfs_asm
    #else
    #define usbd_hw usbd_devfs
    #endif
    #endif

#else
    #error Unsupported STM32 family
#endif

#if defined (__cplusplus)
    }
#endif
#endif //_USB_H_
