

/*
 * Initialize USB ACM serial device, provide convenience
 * function for serial communication.
 *
 * Mostly plucked together from some opencm3 example code.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/nvic.h>

#include "usb.h"

#define STATUS_LED_PORT GPIOC
#define STATUS_LED_PIN  GPIO13

#define USBD_PORT GPIOA
#define USBDM     GPIO11
#define USBDP     GPIO12

#define RX_ECHO     1
#define RX_BUF_LEN  256

static usbd_device *usbd_dev;

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static const struct usb_device_descriptor device_descriptor = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0xfe,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x1209,
    .idProduct = 0x6827,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};


static const char *usb_strings[] = {
    "Lem Cyber Research Institute (LCRI)",
    "bluepoke",
    "bp1-0000",
};


void usb_gpio_init() {
    // Clock
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);

    // D+
    gpio_set_mode(USBD_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  USBDP);

    // LED
    gpio_set_mode(STATUS_LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  STATUS_LED_PIN);

}

void usb_status_led_toggle() {
    gpio_toggle(STATUS_LED_PORT, STATUS_LED_PIN);
}

#define SAMPLE_BUF_LEN (SAMPLE_BUF_SIZE/sizeof(uint32_t))

struct {
    uint32_t buf[SAMPLE_BUF_LEN];
    uint32_t buf_end[0];
} sample __attribute__((section(".sample")));

void sample_write_tile_pattern(uint8_t *tile, size_t len) {
    uint8_t *target=(uint8_t *)sample.buf;

    while (target < (uint8_t *)sample.buf_end) {
        uint8_t *source=tile, *source_end=tile+len;
        while (source < source_end && target < (uint8_t *)sample.buf_end)
            *target++ = *source++;
    }
}

void sample_measure_tile_pattern(uint8_t *tile, size_t len) {
    uint8_t *target=(uint8_t *)sample.buf;

    while (target < (uint8_t *)sample.buf_end) {
        uint8_t *source=tile, *source_end=tile+len;
        while (source < source_end && target < (uint8_t *)sample.buf_end)
            *target++ ^= *source++;
    }
}

uint32_t xorshift128(uint32_t state[static 4]) {
    uint32_t s, t=state[3];
    t ^= t<<11;
    t ^= t>>8;
    state[3]     = state[2];
    state[2]     = state[1];
    state[1] = s = state[0];
    t ^= s;
    t ^= s>>19;
    state[0] = t;
    return t;
}

void sample_write_indices() {
    uint32_t *target = sample.buf, i = 0;
    while (target < sample.buf_end)
        *target++ = i++;
}

void sample_write_xorshift_pattern(uint32_t seed) {
    uint32_t xs_state[4] = {0xdeadcafe, 1, 0, seed};
    
    uint32_t *target = sample.buf;
    while (target < sample.buf_end)
        *target++ = xorshift128(xs_state);
}

void sample_measure_xorshift_pattern(uint32_t seed) {
    uint32_t xs_state[4] = {0xdeadcafe, 1, 0, seed};
    
    uint32_t *target = sample.buf;
    while (target < sample.buf_end)
        *target++ ^= xorshift128(xs_state);
}

enum control_request_type {
    REQ_TYPE_WRITE = 0xd0,
    REQ_TYPE_READ  = 0xf0,
    REQ_TYPE_MASK  = 0xf0,
};

enum control_request {
    REQ_SET_READ_OFFX   = 0x01,
    REQ_MEAS_TEMP       = 0x02,
    REQ_WRITE_TILE      = REQ_TYPE_WRITE | 1,
    REQ_WRITE_XORSHIFT  = REQ_TYPE_WRITE | 2,
    REQ_WRITE_INDICES   = REQ_TYPE_WRITE | 3,
    REQ_MEAS_TILE       = REQ_TYPE_READ  | 1,
    REQ_MEAS_XORSHIFT   = REQ_TYPE_READ  | 2,
};

static size_t sample_next_idx = 0;

static size_t enqueue_read_packet(usbd_device *usbd_dev, uint8_t ep, size_t idx) {
    if (idx > sizeof(sample.buf)) /* Do not read past end of buffer */
        return idx;

    int len = sizeof(sample.buf) - idx;
    if (len > 64) /* Cap to max USB packet size */
        len = 64;

    len = usbd_ep_write_packet(usbd_dev, ep, ((uint8_t *)sample.buf) + idx, len);
    return idx + len;
}

static enum usbd_request_return_codes usb_control_request_cb(usbd_device *usbd_dev, struct usb_setup_data *req,
        uint8_t **buf, uint16_t *len, usbd_control_complete_callback *complete) {
    static uint16_t temp_buf;
    (void)usbd_dev;
    (void)buf;
    (void)len;
    (void)complete;

    switch (req->bRequest) {
        case REQ_SET_READ_OFFX:
            usbd_ep_stall_set(usbd_dev, 0x82, 0); /* This resets the endpoint data buffer */
            sample_next_idx = enqueue_read_packet(usbd_dev, 0x82, req->wIndex);
            break;
        case REQ_WRITE_TILE:        sample_write_tile_pattern  (*buf, *len); break;
        case REQ_MEAS_TILE:         sample_measure_tile_pattern(*buf, *len); break;
        case REQ_WRITE_XORSHIFT:    sample_write_xorshift_pattern  (((uint32_t)req->wIndex)<<16 | req->wLength); break;
        case REQ_MEAS_XORSHIFT:     sample_measure_xorshift_pattern(((uint32_t)req->wIndex)<<16 | req->wLength); break;
        case REQ_WRITE_INDICES:     sample_write_indices(); break;
        case REQ_MEAS_TEMP:
            temp_buf = 0x2342; /* FIXME todo */
            *buf = (uint8_t *)&temp_buf;
            *len = sizeof(temp_buf);
            break;
        default: return 0; /* FIXME barf here */
    }

    return 1;
}

static void usb_data_tx_cb(usbd_device *usbd_dev, uint8_t ep) {
    sample_next_idx = enqueue_read_packet(usbd_dev, ep, sample_next_idx);
}

static void usb_set_config_cb(usbd_device *usbd_dev, uint16_t wValue) {
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, usb_data_tx_cb);

    usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        usb_control_request_cb
    );
}

usbd_device *usb_serial_init() {
    // Initialize GPIO
    usb_gpio_init();

    // Pull down D+
    gpio_clear(USBD_PORT, USBDP);
    for (int i = 0; i < 0x10000; i++) {
        __asm__("nop");
    }
    gpio_set(USBD_PORT, USBDP);

    // Initialize usb device
    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver,
                         &device_descriptor,
                         &config,
                         usb_strings, 3,
                         usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, usb_set_config_cb);

    // Pull down
    gpio_clear(USBD_PORT, USBDP);

    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ);

    return usbd_dev;
}

void usb_lp_can_rx0_isr() {
    usbd_poll(usbd_dev);
    nvic_clear_pending_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}

void usb_hp_can_tx_isr() {
    usbd_poll(usbd_dev);
    nvic_clear_pending_irq(NVIC_USB_HP_CAN_TX_IRQ);
}
