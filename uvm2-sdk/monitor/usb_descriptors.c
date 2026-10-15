/* The monitor's USB identity: a CDC serial port with its own product id, so it is told
 * apart from the multicart menu's (Raspberry Pi's 0x0009 "Pico"). */
#include "tusb.h"

#define USB_VID 0x2E8A
#define USB_PID 0x10DB   /* "debug" */

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE, .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC, .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, .idVendor = USB_VID, .idProduct = USB_PID, .bcdDevice = 0x0100,
    .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3, .bNumConfigurations = 1,
};
uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&desc_device; }

enum { ITF_CDC = 0, ITF_CDC_DATA, ITF_TOTAL };
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
static const uint8_t desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(ITF_CDC, 4, 0x81, 8, 0x02, 0x82, 64),
};
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) { (void)index; return desc_config; }

static const char *const strings[] = { "", "Vectrex Studio", "UVMC2 debug monitor", "1", "Monitor" };
static uint16_t desc_str[32];
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t n;
    if (index == 0) { desc_str[1] = 0x0409; n = 1; }
    else {
        if (index >= sizeof strings / sizeof strings[0]) return 0;
        const char *s = strings[index];
        for (n = 0; s[n] && n < 31; n++) desc_str[1 + n] = (uint16_t)s[n];
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return desc_str;
}
