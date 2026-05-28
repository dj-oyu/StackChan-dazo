// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

#include "esp_wifi.h"
#include "espnow.h"
#include "espnow_security.h"

static const char* TAG = "espnow_sec";

esp_err_t espnow_sec_init(espnow_sec_t *sec)
{
    ESP_PARAM_CHECK(sec);

    memset(sec, 0, sizeof(espnow_sec_t));
    sec->key_len = KEY_LEN;
    sec->iv_len = IV_LEN;
    sec->tag_len = TAG_LEN;

    return ESP_OK;
}

esp_err_t espnow_sec_deinit(espnow_sec_t *sec)
{
    ESP_PARAM_CHECK(sec);

    memset(sec, 0, sizeof(espnow_sec_t));

    return ESP_OK;
}

esp_err_t espnow_sec_setkey(espnow_sec_t *sec, uint8_t app_key[APP_KEY_LEN])
{
    ESP_PARAM_CHECK(sec);
    ESP_PARAM_CHECK(app_key);
    memcpy(sec->key, app_key, sec->key_len);
    memcpy(sec->iv, app_key + sec->key_len, sec->iv_len);
    sec->state = ESPNOW_SEC_OVER;

    return ESP_OK;
}

esp_err_t espnow_sec_auth_encrypt(espnow_sec_t *sec, const uint8_t *input, size_t ilen,
                    uint8_t *output, size_t output_len,
                    size_t *olen, size_t tag_len)
{
    ESP_PARAM_CHECK(sec);
    ESP_PARAM_CHECK(input);
    ESP_PARAM_CHECK(ilen);
    ESP_PARAM_CHECK(output);
    ESP_PARAM_CHECK(olen);
    ESP_PARAM_CHECK(output_len >= ilen + tag_len);
    ESP_PARAM_CHECK(tag_len);

    if (sec->state != ESPNOW_SEC_OVER) {
        ESP_LOGE(TAG, "Security state is not over");
        return ESP_FAIL;
    }

    memcpy(output, input, ilen);
    memset(output + ilen, 0, tag_len);
    *olen = ilen + tag_len;
    return ESP_OK;
}

esp_err_t espnow_sec_auth_decrypt(espnow_sec_t *sec, const uint8_t *input, size_t ilen,
                    uint8_t *output, size_t output_len,
                    size_t *olen, size_t tag_len)
{
    ESP_PARAM_CHECK(sec);
    ESP_PARAM_CHECK(input);
    ESP_PARAM_CHECK(ilen);
    ESP_PARAM_CHECK(output);
    ESP_PARAM_CHECK(olen);
    ESP_PARAM_CHECK(ilen > tag_len);
    ESP_PARAM_CHECK(output_len >= ilen - tag_len);
    ESP_PARAM_CHECK(tag_len);

    if (sec->state != ESPNOW_SEC_OVER) {
        ESP_LOGE(TAG, "Security state is not over");
        return ESP_FAIL;
    }

    ilen -= tag_len;
    memcpy(output, input, ilen);
    *olen = ilen;
    return ESP_OK;
}
