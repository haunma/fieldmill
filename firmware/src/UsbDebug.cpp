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
// Adapted to C++ and IAR compiler by Mark Haun, 27 Aug 2018
//

#include "UsbDebug.h"
#include "usb.h"
#include "usb_cdc.h"
#include "usb_std.h"
#include "stm32l4xx.h"   // mh: was stm32.h
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


// mh: These should be member variables but there is some problem with linkage
// that I have not been able to resolve...
static usbd_device udev;
static uint32_t    ubuf[0x20];
static uint8_t     fifo[0x200];
static uint32_t    fpos;

static uint8_t Buf[UsbDebug::BUF_SIZE];
static uint32_t WritePtr, ReadPtr;
    

UsbDebug::UsbDebug()
{
}
   

__packed struct cdc_config {
    struct usb_config_descriptor        config;
    struct usb_interface_descriptor     comm;
    struct usb_cdc_header_desc          cdc_hdr;
    struct usb_cdc_call_mgmt_desc       cdc_mgmt;
    struct usb_cdc_acm_desc             cdc_acm;
    struct usb_cdc_union_desc           cdc_union;
    struct usb_endpoint_descriptor      comm_ep;
    struct usb_interface_descriptor     data;
    struct usb_endpoint_descriptor      data_eprx;
    struct usb_endpoint_descriptor      data_eptx;
};

static const struct usb_device_descriptor device_desc = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DTYPE_DEVICE,
    .bcdUSB             = VERSION_BCD(2,0,0),
    .bDeviceClass       = USB_CLASS_CDC,
    .bDeviceSubClass    = USB_SUBCLASS_NONE,
    .bDeviceProtocol    = USB_PROTO_NONE,
    .bMaxPacketSize0    = CDC_EP0_SIZE,
    .idVendor           = 0x0483,
    .idProduct          = 0x5740,
    .bcdDevice          = VERSION_BCD(1,0,0),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = INTSERIALNO_DESCRIPTOR,
    .bNumConfigurations = 1,
};

static const struct cdc_config config_desc = {
    .config = {
        .bLength                = sizeof(struct usb_config_descriptor),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(struct cdc_config),
        .bNumInterfaces         = 2,
        .bConfigurationValue    = 1,
        .iConfiguration         = NO_DESCRIPTOR,
        .bmAttributes           = USB_CFG_ATTR_RESERVED | USB_CFG_ATTR_SELFPOWERED,
        .bMaxPower              = USB_CFG_POWER_MA(100),
    },
    .comm = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_CDC,
        .bInterfaceSubClass     = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol     = USB_CDC_PROTO_V25TER,
        .iInterface             = NO_DESCRIPTOR,
    },
    .cdc_hdr = {
        .bFunctionLength        = sizeof(struct usb_cdc_header_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_HEADER,
        .bcdCDC                 = VERSION_BCD(1,1,0),
    },
    .cdc_mgmt = {
        .bFunctionLength        = sizeof(struct usb_cdc_call_mgmt_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_CALL_MANAGEMENT,
        .bmCapabilities         = 0,
        .bDataInterface         = 1,

    },
    .cdc_acm = {
        .bFunctionLength        = sizeof(struct usb_cdc_acm_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_ACM,
        .bmCapabilities         = 0,
    },
    .cdc_union = {
        .bFunctionLength        = sizeof(struct usb_cdc_union_desc),
        .bDescriptorType        = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType     = USB_DTYPE_CDC_UNION,
        .bMasterInterface0      = 0,
        .bSlaveInterface0       = 1,
    },
    .comm_ep = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_NTF_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = CDC_NTF_SZ,
        .bInterval              = 0xFF,
    },
    .data = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_CDC_DATA,
        .bInterfaceSubClass     = USB_SUBCLASS_NONE,
        .bInterfaceProtocol     = USB_PROTO_NONE,
        .iInterface             = NO_DESCRIPTOR,
    },
    .data_eprx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_RXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_SZ,
        .bInterval              = 0x01,
    },
    .data_eptx = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = CDC_TXD_EP,
        .bmAttributes           = USB_EPTYPE_BULK,
        .wMaxPacketSize         = CDC_DATA_SZ,
        .bInterval              = 0x01,
    },
};

// mh: rewrote the following three initializations to not use the variadic macro,
// due to differences between C99 and C++11
static const struct usb_string_descriptor lang_desc = {
    .bLength = 4,
    .bDescriptorType = USB_DTYPE_STRING,
    .wString = USB_LANGID_ENG_US
};

static const struct usb_string_descriptor manuf_desc_en = {
    .bLength = sizeof(u"Open source USB stack for STM32"),
    .bDescriptorType = USB_DTYPE_STRING,
    .wString = u"Open source USB stack for STM32"
};

static const struct usb_string_descriptor prod_desc_en = {
    .bLength = sizeof(u"CDC Loopback demo"),
    .bDescriptorType = USB_DTYPE_STRING,
    .wString = u"CDC Loopback demo"
};

static const struct usb_string_descriptor *const dtable[] = {
    &lang_desc,
    &manuf_desc_en,
    &prod_desc_en,
};

static struct usb_cdc_line_coding cdc_line = {
    .dwDTERate          = 9600,  //38400
    .bCharFormat        = USB_CDC_1_STOP_BITS,
    .bParityType        = USB_CDC_NO_PARITY,
    .bDataBits          = 8,
};



usbd_respond UsbDebug::cdc_getdesc(usbd_ctlreq *req, void **address, uint16_t *length) {
    const uint8_t dtype = req->wValue >> 8;
    const uint8_t dnumber = req->wValue & 0xFF;
    const void* desc;
    uint16_t len = 0;
    switch (dtype) {
    case USB_DTYPE_DEVICE:
        desc = &device_desc;
        break;
    case USB_DTYPE_CONFIGURATION:
        desc = &config_desc;
        len = sizeof(config_desc);
        break;
    case USB_DTYPE_STRING:
        if (dnumber < 3) {
            desc = dtable[dnumber];
        } else {
            return usbd_fail;
        }
        break;
    default:
        return usbd_fail;
    }
    if (len == 0) {
        len = ((struct usb_header_descriptor*)desc)->bLength;
    }
    *address = (void*)desc;
    *length = len;
    return usbd_ack;
};


usbd_respond UsbDebug::cdc_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) != (USB_REQ_INTERFACE | USB_REQ_CLASS)) return usbd_fail;
    switch (req->bRequest) {
    case USB_CDC_SET_CONTROL_LINE_STATE:
        return usbd_ack;
    case USB_CDC_SET_LINE_CODING:
        memmove( req->data, &cdc_line, sizeof(cdc_line));
        return usbd_ack;
    case USB_CDC_GET_LINE_CODING:
        dev->status.data_ptr = &cdc_line;
        dev->status.data_count = sizeof(cdc_line);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}


void UsbDebug::cdc_rxonly(usbd_device *dev, uint8_t event, uint8_t ep) {
   usbd_ep_read(dev, ep, fifo, CDC_DATA_SZ);
}

void UsbDebug::cdc_txonly(usbd_device *dev, uint8_t event, uint8_t ep) {
//    uint8_t _t = dev->driver->frame_no();
//    memset(fifo, _t, CDC_DATA_SZ);

//    static uint8_t count = 0;
//    for (int k = 0 ; k < CDC_DATA_SZ ; k++)
//    {
//        fifo[k] = count;
//        count += 11;
//    }
//    usbd_ep_write(dev, ep, fifo, CDC_DATA_SZ);

    uint32_t bytesToRead = (WritePtr - ReadPtr) & BUF_SIZE_MASK;
    if (bytesToRead > CDC_DATA_SZ)
    {
        bytesToRead = CDC_DATA_SZ;
    }
    
    uint32_t bytesUntilWrap = BUF_SIZE - ReadPtr;
    if (bytesToRead > bytesUntilWrap)
    {
        uint8_t tmp[CDC_DATA_SZ];
        memcpy(&tmp[0],              &Buf[ReadPtr], bytesUntilWrap);
        memcpy(&tmp[bytesUntilWrap], &Buf[0],       bytesToRead-bytesUntilWrap);
        usbd_ep_write(dev, ep, &tmp[0], bytesToRead);
    }
    else
    {
        usbd_ep_write(dev, ep, &Buf[ReadPtr], bytesToRead);
    }
    
    ReadPtr = (ReadPtr + bytesToRead) & BUF_SIZE_MASK;
}

void UsbDebug::cdc_loopback(usbd_device *dev, uint8_t event, uint8_t ep) {
    int _t;
    switch (event) {
    case usbd_evt_eptx:
        _t = usbd_ep_write(dev, CDC_TXD_EP, &fifo[0], (fpos < CDC_DATA_SZ) ? fpos : CDC_DATA_SZ);
        if (_t > 0) {
            memmove(&fifo[0], &fifo[_t], fpos - _t);
            fpos -= _t;
        }
    case usbd_evt_eprx:
        if (fpos < (sizeof(fifo) - CDC_DATA_SZ)) {
            _t = usbd_ep_read(dev, CDC_RXD_EP, &fifo[fpos], CDC_DATA_SZ);
            if (_t > 0) {
                fpos += _t;
            }
        }
    default:
        break;
    }
}

usbd_respond UsbDebug::cdc_setconf(usbd_device *dev, uint8_t cfg) {
    switch (cfg) {
    case 0:
        /* deconfiguring device */
        usbd_ep_deconfig(dev, CDC_NTF_EP);
        usbd_ep_deconfig(dev, CDC_TXD_EP);
        usbd_ep_deconfig(dev, CDC_RXD_EP);
        usbd_reg_endpoint(dev, CDC_RXD_EP, 0);
        usbd_reg_endpoint(dev, CDC_TXD_EP, 0);
        return usbd_ack;
    case 1:
        /* configuring device */
        usbd_ep_config(dev, CDC_RXD_EP, USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF, CDC_DATA_SZ);
        usbd_ep_config(dev, CDC_TXD_EP, USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF, CDC_DATA_SZ);
        usbd_ep_config(dev, CDC_NTF_EP, USB_EPTYPE_INTERRUPT, CDC_NTF_SZ);
#if defined(CDC_LOOPBACK)
        usbd_reg_endpoint(dev, CDC_RXD_EP, cdc_loopback);
        usbd_reg_endpoint(dev, CDC_TXD_EP, cdc_loopback);
#else
        usbd_reg_endpoint(dev, CDC_RXD_EP, cdc_rxonly);
        usbd_reg_endpoint(dev, CDC_TXD_EP, cdc_txonly);
#endif
        usbd_ep_write(dev, CDC_TXD_EP, 0, 0);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}


void UsbDebug::cdc_init_usbd() {
    WritePtr = 0;
    ReadPtr = 0;
    
    usbd_init(&udev, &usbd_hw, CDC_EP0_SIZE, ubuf, sizeof(ubuf));
    usbd_reg_config(&udev, cdc_setconf);
    usbd_reg_control(&udev, cdc_control);
    usbd_reg_descr(&udev, cdc_getdesc);

#if defined(CDC_USE_IRQ)
    NVIC_EnableIRQ(OTG_FS_IRQn);
#endif
    
    usbd_enable(&udev, true);
    usbd_connect(&udev, true);
}


int UsbDebug::Write(void *data, int bytesToWrite)
{
    uint32_t bytesAvail = (ReadPtr - WritePtr - 1) & BUF_SIZE_MASK;
    if (bytesToWrite > bytesAvail)
    {
        bytesToWrite = bytesAvail;
    }
    
    uint32_t bytesUntilWrap = BUF_SIZE - WritePtr;
    if (bytesToWrite > bytesUntilWrap)
    {
        memcpy(&Buf[WritePtr], data,                              bytesUntilWrap);
        memcpy(&Buf[0],        (uint8_t *) data + bytesUntilWrap, bytesToWrite-bytesUntilWrap);
    }
    else
    {
        memcpy(&Buf[WritePtr], data, bytesToWrite);
    }
    
    WritePtr = (WritePtr + bytesToWrite) & BUF_SIZE_MASK;
    return bytesToWrite;
}


void UsbDebug::Poll()
{
    usbd_poll(&udev);
}