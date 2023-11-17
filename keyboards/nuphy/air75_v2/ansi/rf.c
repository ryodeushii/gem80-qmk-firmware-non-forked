// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later

#include "ansi.h"
#include "uart.h" // qmk uart.h
#include "stm32f0xx.h"

USART_MGR_STRUCT Usart_Mgr;
#define RX_SBYTE Usart_Mgr.RXDBuf[0]
#define RX_CMD Usart_Mgr.RXDBuf[1]
#define RX_ACK Usart_Mgr.RXDBuf[2]
#define RX_LEN Usart_Mgr.RXDBuf[3]
#define RX_DAT Usart_Mgr.RXDBuf[4]

bool f_uart_ack        = 0;
bool f_rf_read_data_ok = 0;
bool f_rf_sts_sysc_ok  = 0;
bool f_rf_new_adv_ok   = 0;
bool f_rf_reset        = 0;
bool f_rf_hand_ok      = 0;
bool f_goto_sleep      = 0;
bool f_wakeup_prepare  = 0;

uint8_t  func_tab[32]         = {0};
uint8_t  bitkb_report_buf[32] = {0};
uint8_t  bytekb_report_buf[8] = {0};
uint8_t  mouse_report_buf[5]  = {0};
uint16_t conkb_report         = 0;
uint16_t syskb_report         = 0;
uint8_t  sync_lost            = 0;
uint8_t  disconnect_delay     = 0;

extern DEV_INFO_STRUCT dev_info;
extern host_driver_t  *m_host_driver;
extern uint8_t         host_mode;
extern uint8_t         rf_blink_cnt;
extern uint16_t        rf_link_show_time;
extern uint16_t        rf_linking_time;
extern uint16_t        no_act_time;
extern bool            f_send_channel;
extern bool            f_dial_sw_init_ok;

report_mouse_t mousekey_get_report(void);
void           uart_init(uint32_t baud); // qmk uart.c
void           uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size);
void           UART_Send_Bytes(uint8_t *Buffer, uint32_t Length);
uint8_t        get_checksum(uint8_t *buf, uint8_t len);
void           uart_receive_pro(void);
void           break_all_key(void);
uint16_t       host_last_consumer_usage(void);

/**
 * @brief  Uart send consumer keys report.
 * @note Call in host.c
 */
void uart_send_consumer_report(void) {
    no_act_time  = 0;
    conkb_report = host_last_consumer_usage();
    uart_send_report(CMD_RPT_CONSUME, (uint8_t *)(&conkb_report), 2);
}

/**
 * @brief  Uart send mouse keys report.
 * @note Call in host.c
 */
void uart_send_mouse_report(void) {
    report_mouse_t mouse_report = mousekey_get_report();

    no_act_time = 0;
    memcpy(mouse_report_buf, &mouse_report.buttons, 5);
    uart_send_report(CMD_RPT_MS, mouse_report_buf, 5);
}

/**
 * @brief  Uart send system keys report.
 * @note Call in host.c
 */
void uart_send_system_report(void) {
    no_act_time  = 0;
    syskb_report = host_last_system_usage();
    uart_send_report(CMD_RPT_SYS, (uint8_t *)(&syskb_report), 2);
}

/**
 * @brief  Uart send keys report.
 * @note Call in host.c
 */
void uart_send_report_func(void) {
    static bool     f_bitkb_active  = 0;
    static bool     f_bytekb_active = 0;
    static uint32_t interval_timer  = 0;
    report_mouse_t  mouse_report    = mousekey_get_report();

    if (dev_info.link_mode == LINK_USB) return;
    keyboard_protocol          = 1;
    keyboard_report->nkro.mods = get_mods() | get_weak_mods();

    if ((dev_info.sys_sw_state == SYS_SW_MAC) && (memcmp(bytekb_report_buf, keyboard_report->raw, 8))) {
        no_act_time             = 0;
        f_bytekb_active         = 1;
        keyboard_report->raw[1] = 0;
        memcpy(bytekb_report_buf, keyboard_report->raw, 8);
        uart_send_report(CMD_RPT_BYTE_KB, bytekb_report_buf, 8);
    }

    else if ((dev_info.sys_sw_state == SYS_SW_WIN) && (memcmp(bitkb_report_buf, &keyboard_report->nkro.mods, 16))) {
        no_act_time    = 0;
        f_bitkb_active = 1;
        memcpy(&bitkb_report_buf[0], &keyboard_report->nkro.mods, 16);
        uart_send_report(CMD_RPT_BIT_KB, bitkb_report_buf, 16);
    }

    else if (conkb_report != host_last_consumer_usage()) {
        uart_send_consumer_report();
    }

    else if (syskb_report != host_last_system_usage()) {
        uart_send_system_report();
    }

    else if (memcmp(mouse_report_buf, &mouse_report.buttons, 5)) {
        uart_send_mouse_report();
    }

    else if (timer_elapsed32(interval_timer) > 10) {
        interval_timer = timer_read32();
        if (no_act_time <= 20) {
            if (f_bytekb_active) {
                uart_send_report(CMD_RPT_BYTE_KB, bytekb_report_buf, 8);
                wait_ms(1);
            }

            if (f_bitkb_active) {
                uart_send_report(CMD_RPT_BIT_KB, bitkb_report_buf, 16);
                wait_ms(1);
            }
        } else {
            f_bytekb_active = 0;
            f_bitkb_active  = 0;
        }
    }
}

/**
 * @brief  Parsing the data received from the RF module.
 */
void RF_Protocol_Receive(void) {
    uint8_t i, check_sum = 0;

    if (Usart_Mgr.RXDState == RX_Done) {
        f_uart_ack = 1;

        if (Usart_Mgr.RXDLen > 4) {
            for (i = 0; i < RX_LEN; i++)
                check_sum += Usart_Mgr.RXDBuf[4 + i];

            if (check_sum != Usart_Mgr.RXDBuf[4 + i]) {
                Usart_Mgr.RXDState = RX_SUM_ERR;
                return;
            }
        } else if (Usart_Mgr.RXDLen == 3) {
            if (Usart_Mgr.RXDBuf[2] == 0xA0) {
                f_uart_ack = 1;
            }
        }

        sync_lost = 0;

        switch (RX_CMD) {
            case CMD_HAND: {
                f_rf_hand_ok = 1;
                break;
            }

            case CMD_24G_SUSPEND: {
                f_goto_sleep = 1;
                break;
            }

            case CMD_NEW_ADV: {
                f_rf_new_adv_ok = 1;
                break;
            }

            case CMD_RF_STS_SYSC: {
                static uint8_t error_cnt = 0;

                if (dev_info.link_mode == Usart_Mgr.RXDBuf[4]) {
                    error_cnt = 0;

                    dev_info.rf_state = Usart_Mgr.RXDBuf[5];

                    if ((dev_info.rf_state == RF_CONNECT) && ((Usart_Mgr.RXDBuf[6] & 0xf8) == 0)) {
                        dev_info.rf_led = Usart_Mgr.RXDBuf[6];
                    }

                    dev_info.rf_charge = Usart_Mgr.RXDBuf[7];

                    if (Usart_Mgr.RXDBuf[8] <= 100) dev_info.rf_baterry = Usart_Mgr.RXDBuf[8];
                    if (dev_info.rf_charge & 0x01) dev_info.rf_baterry = 100;
                }

                else {
                    if (dev_info.rf_state != RF_INVAILD) {
                        if (error_cnt >= 5) {
                            error_cnt      = 0;
                            f_send_channel = 1;
                        } else {
                            error_cnt++;
                        }
                    }
                }

                f_rf_sts_sysc_ok = 1;
                break;
            }

            case CMD_READ_DATA: {
                memcpy(func_tab, &Usart_Mgr.RXDBuf[4], 32);

                if (func_tab[4] <= LINK_USB) {
                    dev_info.link_mode = func_tab[4];
                }

                if (func_tab[5] < LINK_USB) {
                    dev_info.rf_channel = func_tab[5];
                }

                if ((func_tab[6] <= LINK_BT_3) && (func_tab[6] >= LINK_BT_1)) {
                    dev_info.ble_channel = func_tab[6];
                }

                f_rf_read_data_ok = 1;
                break;
            }
        }

        Usart_Mgr.RXDLen      = 0;
        Usart_Mgr.RXDState    = RX_Idle;
        Usart_Mgr.RXDOverTime = 0;
    }
}

/**
 * @brief  Uart send cmd.
 * @param  cmd: cmd.
 * @param  wait_ack: wait time for ack after sending.
 * @param  delayms: delay before sending.
 */
uint8_t uart_send_cmd(uint8_t cmd, uint8_t wait_ack, uint8_t delayms) {
    wait_ms(delayms);

    memset(&Usart_Mgr.TXDBuf[0], 0, UART_MAX_LEN);

    Usart_Mgr.TXDBuf[0] = UART_HEAD;
    Usart_Mgr.TXDBuf[1] = cmd;
    Usart_Mgr.TXDBuf[2] = 0x00;

    switch (cmd) {
        case CMD_SLEEP: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = 0;
            Usart_Mgr.TXDBuf[5] = 0;
            break;
        }

        case CMD_HAND: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = 0;
            Usart_Mgr.TXDBuf[5] = 0;
            break;
        }

        case CMD_RF_STS_SYSC: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;
            Usart_Mgr.TXDBuf[5] = dev_info.link_mode;
            break;
        }

        case CMD_SET_LINK: {
            dev_info.rf_state   = RF_LINKING;
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;
            Usart_Mgr.TXDBuf[5] = dev_info.link_mode;

            rf_linking_time  = 0;
            disconnect_delay = 0xff;
            break;
        }

        case CMD_NEW_ADV: {
            dev_info.rf_state   = RF_PAIRING;
            Usart_Mgr.TXDBuf[3] = 2;
            Usart_Mgr.TXDBuf[4] = dev_info.link_mode;
            Usart_Mgr.TXDBuf[5] = 1;
            Usart_Mgr.TXDBuf[6] = dev_info.link_mode + 1;

            rf_linking_time  = 0;
            disconnect_delay = 0xff;
            f_rf_new_adv_ok  = 0;
            break;
        }

        case CMD_CLR_DEVICE: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = 0;
            Usart_Mgr.TXDBuf[5] = 0;
            break;
        }

        case CMD_SET_CONFIG: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = POWER_DOWN_DELAY;
            Usart_Mgr.TXDBuf[5] = POWER_DOWN_DELAY;
            break;
        }

        case CMD_SET_NAME: {
            Usart_Mgr.TXDBuf[3]  = 17;
            Usart_Mgr.TXDBuf[4]  = 1;
            Usart_Mgr.TXDBuf[5]  = 15;
            Usart_Mgr.TXDBuf[6]  = 'N';
            Usart_Mgr.TXDBuf[7]  = 'u';
            Usart_Mgr.TXDBuf[8]  = 'P';
            Usart_Mgr.TXDBuf[9]  = 'h';
            Usart_Mgr.TXDBuf[10] = 'y';
            Usart_Mgr.TXDBuf[11] = ' ';
            Usart_Mgr.TXDBuf[12] = 'A';
            Usart_Mgr.TXDBuf[13] = 'i';
            Usart_Mgr.TXDBuf[14] = 'r';
            Usart_Mgr.TXDBuf[15] = '7';
            Usart_Mgr.TXDBuf[16] = '5';
            Usart_Mgr.TXDBuf[17] = ' ';
            Usart_Mgr.TXDBuf[18] = 'V';
            Usart_Mgr.TXDBuf[19] = '2';
            Usart_Mgr.TXDBuf[20] = '-';
            Usart_Mgr.TXDBuf[21] = get_checksum(Usart_Mgr.TXDBuf + 4, Usart_Mgr.TXDBuf[3]);
            break;
        }

        case CMD_READ_DATA: {
            Usart_Mgr.TXDBuf[3] = 2;
            Usart_Mgr.TXDBuf[4] = 0x00;
            Usart_Mgr.TXDBuf[5] = FUNC_VALID_LEN;
            Usart_Mgr.TXDBuf[6] = FUNC_VALID_LEN;
            break;
        }

        case CMD_RF_DFU: {
            Usart_Mgr.TXDBuf[3] = 1;
            Usart_Mgr.TXDBuf[4] = 0;
            Usart_Mgr.TXDBuf[5] = 0;
            break;
        }

        default:
            break;
    }

    f_uart_ack = 0;
    UART_Send_Bytes(Usart_Mgr.TXDBuf, Usart_Mgr.TXDBuf[3] + 5);

    if (wait_ack) {
        while (wait_ack--) {
            wait_ms(1);
            if (f_uart_ack) return TX_OK;
        }
    } else {
        return TX_OK;
    }

    return TX_TIMEOUT;
}

/**
 * @brief RF module state sync.
 */
void dev_sts_sync(void) {
    static uint32_t interval_timer  = 0;
    static uint8_t  link_state_temp = RF_DISCONNECT;

    if (timer_elapsed32(interval_timer) < 200)
        return;
    else
        interval_timer = timer_read32();

    if (f_rf_reset) {
        f_rf_reset = 0;
        wait_ms(100);
        writePinLow(NRF_RESET_PIN);
        wait_ms(50);
        writePinHigh(NRF_RESET_PIN);
        wait_ms(50);
    }

    else if (f_send_channel) {
        f_send_channel = 0;
        uart_send_cmd(CMD_SET_LINK, 10, 10);
    }

    if (dev_info.link_mode == LINK_USB) {
        if (host_mode != HOST_USB_TYPE) {
            host_mode = HOST_USB_TYPE;
            host_set_driver(m_host_driver);
            break_all_key();
        }
        rf_blink_cnt = 0;
    }

    else {
        if (host_mode != HOST_RF_TYPE) {
            host_mode = HOST_RF_TYPE;
            break_all_key();
            host_set_driver(0);
        }

        if (dev_info.rf_state != RF_CONNECT) {
            if (disconnect_delay >= 10) {
                rf_blink_cnt    = 3;
                link_state_temp = dev_info.rf_state;
            } else {
                disconnect_delay++;
            }
        }

        else if (dev_info.rf_state == RF_CONNECT) {
            rf_linking_time  = 0;
            disconnect_delay = 0;
            rf_blink_cnt     = 0;

            if (link_state_temp != RF_CONNECT) {
                link_state_temp   = RF_CONNECT;
                rf_link_show_time = 0;
            }
        }
    }

    if (f_wakeup_prepare || f_goto_sleep) return;

    uart_send_cmd(CMD_RF_STS_SYSC, 1, 1);

    if (dev_info.link_mode != LINK_USB) {
        if (++sync_lost >= 5) {
            sync_lost  = 0;
            f_rf_reset = 1;
        }
    }
}

/**
 * @brief Uart send bytes.
 * @param Buffer data buf
 * @param Length data lenght
 */
void UART_Send_Bytes(uint8_t *Buffer, uint32_t Length) {
    writePinLow(NRF_WAKEUP_PIN);
    wait_us(50);

    uart_transmit(Buffer, Length);

    wait_us(50 + Length * 30);
    writePinHigh(NRF_WAKEUP_PIN);
}

/**
 * @brief get checksum.
 * @param buf data buf
 * @param len data lenght
 */
uint8_t get_checksum(uint8_t *buf, uint8_t len) {
    uint8_t i;
    uint8_t checksum = 0;

    for (i = 0; i < len; i++)
        checksum += *buf++;

    checksum ^= UART_HEAD;

    return checksum;
}

/**
 * @brief Uart send report.
 * @param report_type  report_type
 * @param report_buf  report_buf
 * @param report_size  report_size
 */
void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size) {
    if (f_dial_sw_init_ok == 0) return;
    if (dev_info.link_mode == LINK_USB) return;
    if (dev_info.rf_state != RF_CONNECT) return;

    Usart_Mgr.TXDBuf[0] = UART_HEAD;
    Usart_Mgr.TXDBuf[1] = report_type;
    Usart_Mgr.TXDBuf[2] = 0x01;
    Usart_Mgr.TXDBuf[3] = report_size;

    memcpy(&Usart_Mgr.TXDBuf[4], report_buf, report_size);
    Usart_Mgr.TXDBuf[4 + report_size] = get_checksum(&Usart_Mgr.TXDBuf[4], report_size);

    UART_Send_Bytes(&Usart_Mgr.TXDBuf[0], report_size + 5);

    wait_us(200);
}

/**
 * @brief Uart receives data and processes it after completion,.
 */
void uart_receive_pro(void) {
    static bool rcv_start = false;

    if (uart_available()) {
        rcv_start = true;
        while (uart_available()) {
            if (Usart_Mgr.RXDLen >= UART_MAX_LEN) {
                uart_read();
                Usart_Mgr.RXDState = RX_DATA_OV;
            } else {
                Usart_Mgr.RXDBuf[Usart_Mgr.RXDLen++] = uart_read();
            }
        }

    } else if (rcv_start) {
        rcv_start          = false;
        Usart_Mgr.RXDState = RX_Done;
        Usart_Mgr.RXDLen   = 0;
        RF_Protocol_Receive();
    }
}

/**
 * @brief  RF uart initial.
 */
void rf_uart_init(void) {
    /* set uart buad as 460800 */
    uart_init(460800);

    /* Enable parity check */
    USART1->CR1 &= ~((uint32_t)USART_CR1_UE);
    USART1->CR1 |= USART_CR1_M0 | USART_CR1_PCE;
    USART1->CR1 |= USART_CR1_UE;

    /* set Rx and Tx pin pull up */
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0);
}

/**
 * @brief RF module initial.
 */
void rf_device_init(void) {
    uint8_t timeout = 0;
    void    uart_receive_pro(void);

    timeout      = 10;
    f_rf_hand_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_HAND, 0, 20);
        wait_ms(5);
        uart_receive_pro(); // receive data
        uart_receive_pro(); // parsing data
        if (f_rf_hand_ok) break;
    }

    timeout           = 10;
    f_rf_read_data_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_READ_DATA, 0, 20);
        wait_ms(5);
        uart_receive_pro();
        uart_receive_pro();
        if (f_rf_read_data_ok) break;
    }

    timeout          = 10;
    f_rf_sts_sysc_ok = 0;
    while (timeout--) {
        uart_send_cmd(CMD_RF_STS_SYSC, 0, 20);
        wait_ms(5);
        uart_receive_pro();
        uart_receive_pro();
        if (f_rf_sts_sysc_ok) break;
    }

    uart_send_cmd(CMD_SET_NAME, 10, 20);
}
