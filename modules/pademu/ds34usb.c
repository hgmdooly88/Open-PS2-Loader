#include "types.h"
#include "loadcore.h"
#include "stdio.h"
#include "sifrpc.h"
#include "sysclib.h"
#include "usbd.h"
#include "usbd_macro.h"
#include "thbase.h"
#include "thsemap.h"
#include "ds34usb.h"
#include "sys_utils.h"
#include "padmacro.h"

#define DPRINTF(x...)
#define REQ_USB_OUT (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define REQ_USB_IN  (USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE)

#define MAX_PADS 4

// 조이트론 스틱 3가지 모드 전체 ID 정의
#define JOYTRON_VID_DI     0x20BC  // DINPUT 제조사 ID
#define JOYTRON_PID_DI     0x5501  // DINPUT 제품 ID
#define JOYTRON_VID_CS     0x0079  // 콘솔/XINPUT 제조사 ID
#define JOYTRON_PID_CS     0x181C  // 콘솔 모드 제품 ID
#define JOYTRON_VID_XI     0x0079  // XINPUT 제조사 ID
#define JOYTRON_PID_XI     0x18A1  // XINPUT 모드 제품 ID

static u8 output_01_report[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
    0xff, 0x27, 0x10, 0x00, 0x32, 0xff, 0x27, 0x10, 0x00, 0x32,
    0xff, 0x27, 0x10, 0x00, 0x32, 0xff, 0x27, 0x10, 0x00, 0x32,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static u8 led_patterns[][2] = {
    {0x1C, 0x02}, {0x1A, 0x04}, {0x16, 0x08}, {0x0E, 0x10}
};

static u8 power_level[] = {
    0x00, 0x00, 0x02, 0x06, 0x0E, 0x1E
};

static u8 rgbled_patterns[][2][3] = {
    {{0x00, 0x00, 0x10}, {0x00, 0x00, 0x7F}},
    {{0x00, 0x10, 0x00}, {0x00, 0x7F, 0x00}},
    {{0x10, 0x10, 0x00}, {0x7F, 0x7F, 0x00}},
    {{0x00, 0x10, 0x10}, {0x00, 0x7F, 0x7F}}
};

static u8 usb_buf[MAX_BUFFER_SIZE + 32] __attribute((aligned(4))) = {0};

int usb_probe(int devId);
int usb_connect(int devId);
int usb_disconnect(int devId);
static void usb_release(int pad);
static void usb_config_set(int result, int count, void *arg);

UsbDriver usb_driver = {NULL, NULL, "ds34usb", usb_probe, usb_connect, usb_disconnect};
static void DS3USB_init(int pad);
static void readReport(u8 *data, int pad);
static int LEDRumble(u8 *led, u8 lrum, u8 rrum, int pad);

ds34usb_device ds34pad[MAX_PADS];

int usb_probe(int devId)
{
    UsbDeviceDescriptor *device = NULL;
    device = (UsbDeviceDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    if (device == NULL) return 0;

    if (device->idVendor == SONY_VID && (device->idProduct == GUITAR_HERO_PS3_PID || device->idProduct == ROCK_BAND_PS3_PID)) return 1;
    if (device->idVendor == DS34_VID && (device->idProduct == DS3_PID || device->idProduct == DS4_PID || device->idProduct == DS4_PID_SLIM)) return 1;
    
    if (device->idVendor == JOYTRON_VID_DI && device->idProduct == JOYTRON_PID_DI) return 1;
    if (device->idVendor == JOYTRON_VID_CS && device->idProduct == JOYTRON_PID_CS) return 1;
    if (device->idVendor == JOYTRON_VID_CS && device->idProduct == JOYTRON_PID_XI) return 1;

    return 0;
}

int usb_connect(int devId)
{
    int pad, epCount;
    UsbDeviceDescriptor *device;
    UsbConfigDescriptor *config;
    UsbInterfaceDescriptor *interface;
    UsbEndpointDescriptor *endpoint;

    for (pad = 0; pad < MAX_PADS; pad++) {
        if (ds34pad[pad].devId == -1 && ds34pad[pad].enabled) break;
    }
    if (pad >= MAX_PADS) return 1;

    PollSema(ds34pad[pad].sema);
    ds34pad[pad].devId = devId;
    ds34pad[pad].status = DS34USB_STATE_AUTHORIZED;
    ds34pad[pad].controlEndp = UsbOpenEndpoint(devId, NULL);

    device = (UsbDeviceDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    config = (UsbConfigDescriptor *)UsbGetDeviceStaticDescriptor(devId, device, USB_DT_CONFIG);
    interface = (UsbInterfaceDescriptor *)((char *)config + config->bLength);

    if (device->idProduct == DS3_PID) {
        ds34pad[pad].type = DS3;
        epCount = interface->bNumEndpoints - 1;
    } else if (device->idProduct == GUITAR_HERO_PS3_PID) {
        ds34pad[pad].type = GUITAR_GH;
        epCount = interface->bNumEndpoints - 1;
    } else if (device->idProduct == ROCK_BAND_PS3_PID) {
        ds34pad[pad].type = GUITAR_RB;
        epCount = interface->bNumEndpoints - 1;
    } else {
        ds34pad[pad].type = DS4;
        epCount = 20;
    }

    endpoint = (UsbEndpointDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_ENDPOINT);
    do {
        if (endpoint->bmAttributes == USB_ENDPOINT_XFER_INT) {
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN && ds34pad[pad].interruptEndp < 0) {
                ds34pad[pad].interruptEndp = UsbOpenEndpointAligned(devId, endpoint);
            }
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT && ds34pad[pad].outEndp < 0) {
                ds34pad[pad].outEndp = UsbOpenEndpointAligned(devId, endpoint);
            }
        }
        endpoint = (UsbEndpointDescriptor *)((char *)endpoint + endpoint->bLength);
    } while (epCount--);

    if (ds34pad[pad].interruptEndp < 0 || ds34pad[pad].outEndp < 0) {
        usb_release(pad);
        return 1;
    }

    ds34pad[pad].status |= DS34USB_STATE_CONNECTED;
    UsbSetDeviceConfiguration(ds34pad[pad].controlEndp, config->bConfigurationValue, usb_config_set, (void *)pad);
    SignalSema(ds34pad[pad].sema);
    return 0;
}

int usb_disconnect(int devId)
{
    u8 pad;
    for (pad = 0; pad < MAX_PADS; pad++) {
        if (ds34pad[pad].devId == devId) break;
    }
    if (pad < MAX_PADS) usb_release(pad);
    return 0;
}

static void usb_release(int pad)
{
    PollSema(ds34pad[pad].sema);
    if (ds34pad[pad].interruptEndp >= 0) UsbCloseEndpoint(ds34pad[pad].interruptEndp);
    if (ds34pad[pad].outEndp >= 0) UsbCloseEndpoint(ds34pad[pad].outEndp);
    ds34pad[pad].controlEndp = -1;
    ds34pad[pad].interruptEndp = -1;
    ds34pad[pad].outEndp = -1;
    ds34pad[pad].devId = -1;
    ds34pad[pad].status = DS34USB_STATE_DISCONNECTED;
    SignalSema(ds34pad[pad].sema);
}

static int usb_resulCode;
static void usb_data_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;
    usb_resulCode = resultCode;
    SignalSema(ds34pad[pad].sema);
}

static void usb_cmd_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;
    SignalSema(ds34pad[pad].cmd_sema);
}

static void usb_config_set(int result, int count, void *arg)
{
    int pad = (int)arg;
    u8 led[4];
    PollSema(ds34pad[pad].sema);
    ds34pad[pad].status |= DS34USB_STATE_CONFIGURED;

    if (ds34pad[pad].type == DS3) {
        DS3USB_init(pad);
        DelayThread(10000);
        led[0] = led_patterns[pad][1];
        led[3] = 0;
    } else if (ds34pad[pad].type == DS4) {
        if (UsbGetDeviceStaticDescriptor(ds34pad[pad].devId, NULL, USB_DT_DEVICE)->idVendor == JOYTRON_VID_DI || 
            UsbGetDeviceStaticDescriptor(ds34pad[pad].devId, NULL, USB_DT_DEVICE)->idVendor == JOYTRON_VID_CS) {
            ds34pad[pad].status |= DS34USB_STATE_RUNNING;
            SignalSema(ds34pad[pad].sema);
            return;
        } else {
            led[0] = rgbled_patterns[pad][1][0];
            led[1] = rgbled_patterns[pad][1][1];
            led[2] = rgbled_patterns[pad][1][2];
            led[3] = 0;
        }
    }

    LEDRumble(led, 0, 0, pad);
    ds34pad[pad].status |= DS34USB_STATE_RUNNING;
    SignalSema(ds34pad[pad].sema);
}

static void DS3USB_init(int pad)
{
    usb_buf[0] = 0x42; usb_buf[1] = 0x0c; usb_buf[2] = 0x00; usb_buf[3] = 0x00;
    UsbControlTransfer(ds34pad[pad].controlEndp, REQ_USB_OUT, USB_REQ_SET_REPORT, (HID_USB_GET_REPORT_FEATURE << 8) | 0xF4, 0, 4, usb_buf, NULL, NULL);
}

#define MAX_DELAY 10
static void readReport(u8 *data, int pad_idx)
{
    ds34usb_device *pad = &ds34pad[pad_idx];
    if (pad->type == GUITAR_GH || pad->type == GUITAR_RB) {
        struct ds3guitarreport *report;
        report = (struct ds3guitarreport *)data;
        translate_pad_guitar(report, &pad->ds2, pad->type == GUITAR_GH);
        padMacroPerf(pad_idx, &pad->ds2);
    } else {
        u8 *report = (u8 *)data;
        if (pad->type == DS3) {
            translate_pad_ds3((struct ds3report *)report, &pad->ds2);
        } else {
            translate_pad_ds4((struct ds4report *)report, &pad->ds2);
        }
        padMacroPerf(pad_idx, &pad->ds2);
    }
}
