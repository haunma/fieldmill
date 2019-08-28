#ifndef __USBDEBUG_H
#define __USBDEBUG_H

#include "usb.h"
#include <stdint.h>

#define CDC_EP0_SIZE    0x08
#define CDC_RXD_EP      0x01
#define CDC_TXD_EP      0x82
#define CDC_DATA_SZ     0x40
#define CDC_NTF_EP      0x83
#define CDC_NTF_SZ      0x08
//#define CDC_LOOPBACK

#define CDC_USE_IRQ   /* uncomment to build interrupt-based demo */


class UsbDebug
{
public:
    enum
    {
        BUF_SIZE_POW2 = 12,
        BUF_SIZE = (1 << BUF_SIZE_POW2),
        BUF_SIZE_MASK = BUF_SIZE - 1,
    };

    
    UsbDebug();
    virtual ~UsbDebug() {};

    // Public API
    static void cdc_init_usbd();
    static void Poll();
    static int Write(void *data, int len);

private:
    // Helpers
    static usbd_respond cdc_setconf(usbd_device *dev, uint8_t cfg);
    static usbd_respond cdc_getdesc(usbd_ctlreq *req, void **address, uint16_t *length);
    static usbd_respond cdc_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback);
    static void cdc_rxonly(usbd_device *dev, uint8_t event, uint8_t ep);
    static void cdc_txonly(usbd_device *dev, uint8_t event, uint8_t ep);
    static void cdc_loopback(usbd_device *dev, uint8_t event, uint8_t ep);
    
    UsbDebug(const UsbDebug&);
    void operator=(const UsbDebug&);
};

extern UsbDebug g_usb;

#endif  // __USBDEBUG_H defined