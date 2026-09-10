#pragma once

/* Minimal host-test substitute for the ESP-IDF header of the same name. Every
 * main/ header that returns an esp_err_t includes it, so the gcc commands in
 * README section 9 put this directory on the include path (-Itests/stubs). */
typedef int esp_err_t;

#define ESP_OK                0
#define ESP_ERR_INVALID_ARG   0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NO_MEM        0x101
