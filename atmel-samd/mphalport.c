#include <string.h>

#include "autoreload.h"
#include "lib/mp-readline/readline.h"
#include "lib/utils/interrupt_char.h"
#include "py/mphal.h"
#include "py/mpstate.h"
#include "py/smallint.h"
#include "shared-bindings/microcontroller/__init__.h"
#include "shared-bindings/time/__init__.h"

#include "hal/include/hal_atomic.h"
#include "hal/include/hal_delay.h"
#include "hal/include/hal_gpio.h"
#include "hal/include/hal_sleep.h"

#include "mpconfigboard.h"
#include "mphalport.h"
#include "usb.h"

extern struct usart_module usart_instance;

// Read by main to know when USB is connected.
volatile bool mp_msc_enabled = false;
bool mp_msc_enable() {
    mp_msc_enabled = true;
    return true;
}

void mp_msc_disable() {
    mp_msc_enabled = false;
}

bool mp_cdc_enable(uint8_t port) {
    mp_cdc_enabled = false;
    return true;
}

void mp_cdc_disable(uint8_t port) {
    mp_cdc_enabled = false;
}

volatile bool reset_on_disconnect = false;

void usb_dtr_notify(uint8_t port, bool set) {
    mp_cdc_enabled = set;
    if (!set && reset_on_disconnect) {
        reset_to_bootloader();
    }
}

void usb_rts_notify(uint8_t port, bool set) {
    return;
}

<<<<<<< HEAD
void usb_coding_notify(uint8_t port, usb_cdc_line_coding_t* coding) {
    reset_on_disconnect = coding->dwDTERate == 1200;
}

void usb_rx_notify(void) {
    irqflags_t flags;
    if (mp_cdc_enabled) {
        while (udi_cdc_is_rx_ready()) {
            uint8_t c;

            atomic_enter_critical(&flags);
            // If our buffer is full, then don't get another character otherwise
            // we'll lose a previous character.
            if (usb_rx_count >= USB_RX_BUF_SIZE) {
                atomic_leave_critical(&flags);
                break;
            }

            uint8_t current_tail = usb_rx_buf_tail;
            // Pretend we've received a character so that any nested calls to
            // this function have to consider the spot we've reserved.
            if ((USB_RX_BUF_SIZE - 1) == usb_rx_buf_tail) {
                // Reached the end of buffer, revert back to beginning of
                // buffer.
                usb_rx_buf_tail = 0x00;
            } else {
                usb_rx_buf_tail++;
            }
            // The count of characters present in receive buffer is
            // incremented.
            usb_rx_count++;
            // WARNING(tannewt): This call can call us back with the next
            // character!
            c = udi_cdc_getc();

            if (c == mp_interrupt_char) {
                // We consumed a character rather than adding it to the rx
                // buffer so undo the modifications we made to count and the
                // tail.
                usb_rx_count--;
                usb_rx_buf_tail = current_tail;
                atomic_leave_critical(&flags);
                mp_keyboard_interrupt();
                // Don't put the interrupt into the buffer, just continue.
                continue;
            }

            // We put the next character where we expected regardless of whether
            // the next character was already loaded in the buffer.
            usb_rx_buf[current_tail] = c;

            atomic_leave_critical(&flags);
        }
    }
}

int receive_usb(void) {
    if (usb_rx_count == 0) {
        return 0;
    }

    // Copy from head.
    int data;
    CRITICAL_SECTION_ENTER();
    data = usb_rx_buf[usb_rx_buf_head];
    usb_rx_buf_head++;
    usb_rx_count--;
    if ((USB_RX_BUF_SIZE) == usb_rx_buf_head) {
      usb_rx_buf_head = 0;
    }
    CRITICAL_SECTION_LEAVE();

    // Call usb_rx_notify if we just emptied a spot in the buffer.
    if (usb_rx_count == USB_RX_BUF_SIZE - 1) {
         usb_rx_notify();
    }
    return data;
}

=======
>>>>>>> more work
int mp_hal_stdin_rx_chr(void) {
    for (;;) {
        #ifdef MICROPY_VM_HOOK_LOOP
            MICROPY_VM_HOOK_LOOP
        #endif
<<<<<<< HEAD
        #ifdef USB_REPL
        if (reload_next_character) {
=======
        if (reset_next_character) {
>>>>>>> more work
            return CHAR_CTRL_D;
        }
        // if (usb_rx_count > 0) {
        //     #ifdef MICROPY_HW_LED_RX
        //     port_pin_toggle_output_level(MICROPY_HW_LED_RX);
        //     #endif
        //     return receive_usb();
        // }

        sleep(3);
    }
}

void mp_hal_stdout_tx_strn(const char *str, size_t len) {
    #ifdef MICROPY_HW_LED_TX
    gpio_toggle_pin_level(MICROPY_HW_LED_TX);
    #endif

    usb_write((const uint8_t*) str, len);

<<<<<<< HEAD
    #ifdef CIRCUITPY_BOOT_OUTPUT_FILE
    if (boot_output_file != NULL) {
        UINT bytes_written = 0;
        f_write(boot_output_file, str, len, &bytes_written);
    }
    #endif

    #ifdef USB_REPL
    // Always make sure there is enough room in the usb buffer for the outgoing
    // string. If there isn't we risk getting caught in a loop within the usb
    // code as it tries to send all the characters it can't buffer.
    uint32_t start = 0;
    uint64_t start_tick = common_hal_time_monotonic();
    uint64_t duration = 0;
    if (mp_cdc_enabled) {
        while (start < len && duration < 10) {
            uint8_t buffer_space = udi_cdc_get_free_tx_buffer();
            uint8_t transmit = min(len - start, buffer_space);
            if (transmit > 0) {
                if (udi_cdc_write_buf(str + start, transmit) > 0) {
                    // It didn't transmit successfully so give up.
                    break;
                }
            }
            start += transmit;
            #ifdef MICROPY_VM_HOOK_LOOP
                MICROPY_VM_HOOK_LOOP
            #endif
            duration = (common_hal_time_monotonic() - start_tick);
        }
    }
    #endif
=======
>>>>>>> more work
}

void mp_hal_delay_ms(mp_uint_t delay) {
    uint64_t start_tick = ticks_ms;
    uint64_t duration = 0;
    while (duration < delay) {
        #ifdef MICROPY_VM_HOOK_LOOP
            MICROPY_VM_HOOK_LOOP
        #endif
        // Check to see if we've been CTRL-Ced by autoreload or the user.
        if(MP_STATE_VM(mp_pending_exception) == MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception))) {
            break;
        }
        duration = (ticks_ms - start_tick);
        // TODO(tannewt): Go to sleep for a little while while we wait.
    }
}

void mp_hal_delay_us(mp_uint_t delay) {
    delay_us(delay);
}

void mp_hal_disable_all_interrupts(void) {
    common_hal_mcu_disable_interrupts();
}

void mp_hal_enable_all_interrupts(void) {
    common_hal_mcu_enable_interrupts();
}
