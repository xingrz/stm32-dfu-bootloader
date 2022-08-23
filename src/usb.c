#include <string.h>

#include "usb.h"

// Defined in main
extern uint8_t usbd_control_buffer[1024];
extern const char * const _usb_strings[5];

// Simple builtin fns
size_t strlen(const char *s) {
	size_t ret = 0;
	while (*s++)
		ret++;
	return ret;
}

const struct usb_device_descriptor dev_desc = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0xdead,
	.idProduct = 0xca5d,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes =
		#ifdef ENABLE_DFU_UPLOAD
		USB_DFU_CAN_UPLOAD |
		#endif
		USB_DFU_CAN_DOWNLOAD |
		USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = DFU_TRANSFER_SIZE,
	.bcdDFUVersion = 0x011A,
};

static const struct usb_interface_descriptor iface_dfu = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_DFU,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,
	.iInterface = 4,
	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

static const struct usb_interface ifaces_dfu = {
	.num_altsetting = 1,
	.altsetting = &iface_dfu,
};

static const struct usb_config_descriptor config[] = {
	{
		.bLength = USB_DT_CONFIGURATION_SIZE,
		.bDescriptorType = USB_DT_CONFIGURATION,
		.wTotalLength = 0,
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 5,
		.bmAttributes = 0xC0,
		.bMaxPower = 0x32,
		.interface = &ifaces_dfu,
	},
};

usbd_device *usbd_dev;

void usb_init(usbd_control_callback callback) {
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_desc, config,
		_usb_strings, 5,
		usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_control_callback(usbd_dev,
		USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		callback);
}

void usb_pwdn() {
#define USB_CNTR_REG (&MMIO32(USB_DEV_FS_BASE + 0x40))
#define USB_CNTR_PWDN 0x0002 /* Power down */
	*USB_CNTR_REG = USB_CNTR_PWDN;
}

void do_usb_poll() {
	usbd_poll(usbd_dev);
}
