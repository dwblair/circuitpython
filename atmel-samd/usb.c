/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>

#include "usb/class/cdc/device/cdcdf_acm.h"
// #include "hiddf_mouse.h"
// #include "hiddf_keyboard.h"
#include "usb/class/hid/device/hiddf_generic.h"
#include "usb/class/composite/device/composite_desc.h"

#include "hal/include/hal_gpio.h"

// // Store received characters on our own so that we can filter control characters
// // and act immediately on CTRL-C for example.
//
// // Receive buffer
// static uint8_t usb_rx_buf[USB_RX_BUF_SIZE];
//
// // Receive buffer head
// static volatile uint8_t usb_rx_buf_head;
//
// // Receive buffer tail
// static volatile uint8_t usb_rx_buf_tail;
//
// // Number of bytes in receive buffer
// volatile uint8_t usb_rx_count;
//
// volatile bool mp_cdc_enabled = false;

// void usb_rx_notify(void)
// {
//     volatile hal_atomic_t flags;
//     if (mp_cdc_enabled) {
//         while (udi_cdc_is_rx_ready()) {
//             uint8_t c;
//
//             atomic_enter_critical(&flags);
//             // If our buffer is full, then don't get another character otherwise
//             // we'll lose a previous character.
//             if (usb_rx_count >= USB_RX_BUF_SIZE) {
//                 atomic_leave_critical(&flags);
//                 break;
//             }
//
//             uint8_t current_tail = usb_rx_buf_tail;
//             // Pretend we've received a character so that any nested calls to
//             // this function have to consider the spot we've reserved.
//             if ((USB_RX_BUF_SIZE - 1) == usb_rx_buf_tail) {
//                 // Reached the end of buffer, revert back to beginning of
//                 // buffer.
//                 usb_rx_buf_tail = 0x00;
//             } else {
//                 usb_rx_buf_tail++;
//             }
//             // The count of characters present in receive buffer is
//             // incremented.
//             usb_rx_count++;
//             // WARNING(tannewt): This call can call us back with the next
//             // character!
//             c = udi_cdc_getc();
//
//             if (c == mp_interrupt_char) {
//                 // We consumed a character rather than adding it to the rx
//                 // buffer so undo the modifications we made to count and the
//                 // tail.
//                 usb_rx_count--;
//                 usb_rx_buf_tail = current_tail;
//                 atomic_leave_critical(&flags);
//                 mp_keyboard_interrupt();
//                 // Don't put the interrupt into the buffer, just continue.
//                 continue;
//             }
//
//             // We put the next character where we expected regardless of whether
//             // the next character was already loaded in the buffer.
//             usb_rx_buf[current_tail] = c;
//
//             atomic_leave_critical(&flags);
//         }
//     }
// }

// int receive_usb(void) {
//     if (usb_rx_count == 0) {
//         return 0;
//     }
//
//     // Disable autoreset if someone is using the repl.
//     autoreset_disable();
//
//     // Copy from head.
//     int data;
//     CRITICAL_SECTION_ENTER();
//     data = usb_rx_buf[usb_rx_buf_head];
//     usb_rx_buf_head++;
//     usb_rx_count--;
//     if ((USB_RX_BUF_SIZE) == usb_rx_buf_head) {
//       usb_rx_buf_head = 0;
//     }
//     CRITICAL_SECTION_LEAVE();
//
//     // Call usb_rx_notify if we just emptied a spot in the buffer.
//     if (usb_rx_count == USB_RX_BUF_SIZE - 1) {
//          usb_rx_notify();
//     }
//     return data;
// }

static uint8_t multi_desc_bytes[] = {
 /* Device descriptors and Configuration descriptors list. */
 COMPOSITE_DESCES_LS_FS,
};

static struct usbd_descriptors multi_desc = {multi_desc_bytes, multi_desc_bytes + sizeof(multi_desc_bytes)};

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

static void init_hardware(void) {
    #ifdef SAMD21
    _pm_enable_bus_clock(PM_BUS_APBB, USB);
    _pm_enable_bus_clock(PM_BUS_AHB, USB);
    _gclk_enable_channel(USB_GCLK_ID, CONF_GCLK_USB_SRC);
    #endif

    #ifdef SAMD51
    hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, CONF_GCLK_USB_SRC | GCLK_PCHCTRL_CHEN);
    hri_mclk_set_AHBMASK_USB_bit(MCLK);
    hri_mclk_set_APBBMASK_USB_bit(MCLK);
    #endif

    usb_d_init();

    gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(PIN_PA24, false);
    gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);
    gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(PIN_PA25, false);
    gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);
    #ifdef SAMD21
    gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
    gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);
    #endif
    #ifdef SAMD51
    gpio_set_pin_function(PIN_PA24, PINMUX_PA24H_USB_DM);
    gpio_set_pin_function(PIN_PA25, PINMUX_PA25H_USB_DP);
    #endif
}

static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{

    /* No error. */
    return false;
}

static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{

    /* No error. */
    return false;
}

static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
    // if (state.rs232.DTR) {
    //     /* Start Rx */
    //     cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, 64);
    // }

    /* No error. */
    return false;
}

// static void usbd_sof_event(void)
// {
//     // Triggers button state checks and HID response.
// }

void init_usb(void) {
    init_hardware();

    usbdc_init(ctrl_buffer);

    /* usbdc_register_funcion inside */
    cdcdf_acm_init();
    // hiddf_mouse_init();
    // hiddf_keyboard_init();

    usbdc_start(&multi_desc);
    usbdc_attach();

    // Maybe wait for USB to be connected.
    cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_out);
    cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
    cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
    //usbdc_register_handler(USBDC_HDL_SOF, &usbd_sof_event_h);
}

void usb_write(const uint8_t* buffer, uint32_t len) {
    // // Always make sure there is enough room in the usb buffer for the outgoing
    // // string. If there isn't we risk getting caught in a loop within the usb
    // // code as it tries to send all the characters it can't buffer.
    // uint32_t start = 0;
    // uint64_t start_tick = common_hal_time_monotonic();
    // uint64_t duration = 0;
    // if (mp_cdc_enabled) {
    //     while (start < len && duration < 10) {
    //         uint8_t buffer_space = udi_cdc_get_free_tx_buffer();
    //         uint8_t transmit = min(len - start, buffer_space);
    //         if (transmit > 0) {
    //             if (udi_cdc_write_buf(str + start, transmit) > 0) {
    //                 // It didn't transmit successfully so give up.
    //                 break;
    //             }
    //         }
    //         start += transmit;
    //         #ifdef MICROPY_VM_HOOK_LOOP
    //             MICROPY_VM_HOOK_LOOP
    //         #endif
    //         duration = (common_hal_time_monotonic() - start_tick);
    //     }
    // }
}
