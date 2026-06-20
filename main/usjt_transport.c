#include "usjt_transport.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef USJT_TX_BUF_SIZE
#define USJT_TX_BUF_SIZE        1024
#endif
#ifndef USJT_RX_BUF_SIZE
#define USJT_RX_BUF_SIZE        1024
#endif
#ifndef USJT_WRITE_TIMEOUT_MS
#define USJT_WRITE_TIMEOUT_MS   100
#endif

bool usjt_transport_open(struct uxrCustomTransport *transport) {
    (void)transport;
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = USJT_TX_BUF_SIZE,
        .rx_buffer_size = USJT_RX_BUF_SIZE,
    };
    return usb_serial_jtag_driver_install(&cfg) == ESP_OK;
}

bool usjt_transport_close(struct uxrCustomTransport *transport) {
    (void)transport;
    return usb_serial_jtag_driver_uninstall() == ESP_OK;
}

size_t usjt_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf,
                             size_t len, uint8_t *err) {
    (void)transport;
    int written = usb_serial_jtag_write_bytes(buf, len,
                    pdMS_TO_TICKS(USJT_WRITE_TIMEOUT_MS));
    if (written <= 0) {
        if (err) *err = 1;
        return 0;
    }
    return (size_t)written;
}

size_t usjt_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                            size_t len, int timeout, uint8_t *err) {
    (void)transport;
    int rd = usb_serial_jtag_read_bytes(buf, (int)len, pdMS_TO_TICKS(timeout));
    if (rd <= 0) {
        if (err) *err = 1;
        return 0;
    }
    return (size_t)rd;
}
