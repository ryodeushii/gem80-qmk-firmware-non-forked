// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#define DEV_MODE_PIN                        C0
#define SYS_MODE_PIN                        C1
#define DC_BOOST_PIN                        C2
#define NRF_RESET_PIN                       B4
#define NRF_TEST_PIN                        B5
#define NRF_WAKEUP_PIN                      B8
#define DRIVER_LED_CS_PIN                   C6

#define DRIVER_SIDE_PIN                     C8
#define DRIVER_SIDE_CS_PIN                  C9

#define SERIAL_DRIVER                       SD1
#define SD1_TX_PIN                          B6
#define SD1_TX_PAL_MODE                     0
#define SD1_RX_PIN                          B7
#define SD1_RX_PAL_MODE                     0

#define TAP_CODE_DELAY                      8
#define DYNAMIC_KEYMAP_MACRO_DELAY          8
#define DYNAMIC_KEYMAP_LAYER_COUNT          8

#define EECONFIG_USER_DATA_SIZE             8

#define RGB_MATRIX_DEFAULT_MODE             RGB_MATRIX_CYCLE_LEFT_RIGHT
#define RGB_DISABLE_WHEN_USB_SUSPENDED
