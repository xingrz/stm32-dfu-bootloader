#pragma once

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>
#include <libopencm3/stm32/f1/memorymap.h>

#define DFU_TRANSFER_SIZE 1024

void usb_init(usbd_control_callback callback);
void usb_pwdn();
void do_usb_poll();
