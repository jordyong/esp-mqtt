#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void uartManager_init();
int sendData(const char *data);

#ifdef __cplusplus
}
#endif
