//*****************************************************************************
//
//! @file tinyusb_webusb.c
//!
//! @brief tinyusb webusb example.
//!
//! This example demonstrates WebUSB as web serial with browser with WebUSB support (e.g Chrome).
//! After enumerated successfully, browser will pop-up notification
//! with URL to landing page, click on it to test
//! - Click "Connect" and select device, When connected the on-board LED will litted up.
//! - Any charters received from either webusb/Serial will be echo back to webusb and Serial
//!
//! Note:
//! - The WebUSB landing page notification is currently disabled in Chrome
//! on Windows due to Chromium issue 656702 (https://crbug.com/656702). You have to
//! go to landing page (below) to test
//!
//! - On Windows 7 and prior: You need to use Zadig tool to manually bind the
//! WebUSB interface with the WinUSB driver for Chrome to access. From windows 8 and 10, this
//!  is done automatically by firmware.
//! - On Linux/macOS, udev permission may need to be updated by
//!   - copying '/examples/device/99-tinyusb.rules' file to /etc/udev/rules.d/ then
//!   - run 'sudo udevadm control --reload-rules && sudo udevadm trigger'
//!
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2022, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_4_1_0-8020bdf229 of the AmbiqSuite Development Package.
//
//*****************************************************************************

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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
 *
 */

/* This example demonstrates WebUSB as web serial with browser with WebUSB support (e.g Chrome).
 * After enumerated successfully, browser will pop-up notification
 * with URL to landing page, click on it to test
 *  - Click "Connect" and select device, When connected the on-board LED will litted up.
 *  - Any charters received from either webusb/Serial will be echo back to webusb and Serial
 *
 * Note:
 * - The WebUSB landing page notification is currently disabled in Chrome
 * on Windows due to Chromium issue 656702 (https://crbug.com/656702). You have to
 * go to landing page (below) to test
 *
 * - On Windows 7 and prior: You need to use Zadig tool to manually bind the
 * WebUSB interface with the WinUSB driver for Chrome to access. From windows 8 and 10, this
 * is done automatically by firmware.
 *
 * - On Linux/macOS, udev permission may need to be updated by
 *   - copying '/examples/device/99-tinyusb.rules' file to /etc/udev/rules.d/ then
 *   - run 'sudo udevadm control --reload-rules && sudo udevadm trigger'
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "am_bsp.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "tusb.h"
#include "tusb_config.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
    BLINK_ALWAYS_ON = UINT32_MAX,
    BLINK_ALWAYS_OFF = 0
};

enum { VENDOR_REQUEST_WEBUSB = 1, VENDOR_REQUEST_MICROSOFT = 2 };

extern uint8_t const desc_ms_os_20[];

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

#define URL "example.tinyusb.org/webusb-serial/"

const tusb_desc_webusb_url_t desc_url = {.bLength = 3 + sizeof(URL) - 1,
                                         .bDescriptorType = 3, // WEBUSB URL type
                                         .bScheme = 1,         // 0: http, 1: https
                                         .url = URL};

static bool web_serial_connected = false;

void
led_blinking_task(void);
void
cdc_task(void);
void
webserial_task(void);

void
board_init(void) {
    am_util_id_t sIdDevice;

    //
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    // am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_debug_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("USB CDC Example\n\n");

    //
    // Print the device info.
    //
    am_util_id_device(&sIdDevice);
    am_util_stdio_printf("Vendor Name: %s\n", sIdDevice.pui8VendorName);
    am_util_stdio_printf("Device type: %s\n\n", sIdDevice.pui8DeviceName);
}

uint32_t
board_millis(void) {
    return 0;
}

void
board_led_write(bool bLedState) {
    return;
}

/*------------- MAIN -------------*/
int
main(void) {
    board_init();
    tusb_init();

    while (1) {
        tud_task(); // tinyusb device task
        led_blinking_task();
        webserial_task();
        cdc_task();
    }

#if 0 // Avoid compiler warning, unreachable statement
    return 0;
#endif
}

// send characters to both CDC and WebUSB
void
echo_all(uint8_t buf[], uint32_t count) {
    // echo to web serial
    if (web_serial_connected) {
        tud_vendor_write(buf, count);
    }

    // echo to cdc
    if (tud_cdc_connected()) {
        for (uint32_t i = 0; i < count; i++) {
            tud_cdc_write_char(buf[i]);

            if (buf[i] == '\r')
                tud_cdc_write_char('\n');
        }
        tud_cdc_write_flush();
    }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void
tud_mount_cb(void) {
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void
tud_umount_cb(void) {
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void
tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void
tud_resume_cb(void) {
    blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// WebUSB use vendor class
//--------------------------------------------------------------------+

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool
tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    // nothing to with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_VENDOR:
        switch (request->bRequest) {
        case VENDOR_REQUEST_WEBUSB:
            // match vendor request in BOS descriptor
            // Get landing page url
            return tud_control_xfer(rhport, request, (void *)&desc_url, desc_url.bLength);

        case VENDOR_REQUEST_MICROSOFT:
            if (request->wIndex == 7) {
                // Get Microsoft OS 2.0 compatible descriptor
                uint16_t total_len;
                memcpy(&total_len, desc_ms_os_20 + 8, 2);

                return tud_control_xfer(rhport, request, (void *)desc_ms_os_20, total_len);
            } else {
                return false;
            }

        default:
            break;
        }
        break;

    case TUSB_REQ_TYPE_CLASS:
        if (request->bRequest == 0x22) {
            // Webserial simulate the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to connect and
            // disconnect.
            web_serial_connected = (request->wValue != 0);

            // Always lit LED if connected
            if (web_serial_connected) {
                board_led_write(true);
                blink_interval_ms = BLINK_ALWAYS_ON;

                tud_vendor_write_str("\r\nTinyUSB WebUSB device example\r\n");
            } else {
                blink_interval_ms = BLINK_MOUNTED;
            }

            // response with status OK
            return tud_control_status(rhport, request);
        }
        break;

    default:
        break;
    }

    // stall unknown request
    return false;
}

void
webserial_task(void) {
    if (web_serial_connected) {
        if (tud_vendor_available()) {
            uint8_t buf[64];
            uint32_t count = tud_vendor_read(buf, sizeof(buf));

            // echo back to both web serial and cdc
            echo_all(buf, count);
        }
    }
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void
cdc_task(void) {
    // connected() check for DTR bit
    // Most but not all terminal client set this when making connection
    // if ( tud_cdc_connected() )
    {
        // connected and there are data available
        if (tud_cdc_available()) {
            uint8_t buf[64];

            // read and echo back
            uint32_t count = tud_cdc_read(buf, sizeof(buf));

            // echo back to both web serial and cdc
            echo_all(buf, count);

            // for ( uint32_t i = 0; i < count; i++ )
            // {
            //     tud_cdc_write_char(buf[i]);

            //     if ( buf[i] == '\r' )
            //     {
            //         tud_cdc_write_char('\n');
            //     }
            // }

            // tud_cdc_write_flush();
        }
    }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void
tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;

    // connected
    if (dtr) {
        // print initial message when connected
        tud_cdc_write_str("\r\nTinyUSB WebUSB device example\r\n");
        tud_cdc_write_flush();
    }
}

// Invoked when CDC interface received data from host
void
tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void
led_blinking_task(void) {
    static uint32_t start_ms = 0;
    static bool led_state = false;

    // Blink every interval ms
    if (board_millis() - start_ms < blink_interval_ms) {
        return; // not enough time
    }
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}
