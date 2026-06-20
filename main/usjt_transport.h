#pragma once

#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

bool usjt_transport_open(struct uxrCustomTransport *transport);
bool usjt_transport_close(struct uxrCustomTransport *transport);
size_t usjt_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf,
                            size_t len, uint8_t *err);
size_t usjt_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                           size_t len, int timeout, uint8_t *err);

#ifdef __cplusplus
}
#endif
