#ifndef ENP_H
#define ENP_H

esp_err_t conf_esp_now_protocol(esp_now_peer_info_t *peer);

esp_err_t send_espnow_protocol(esp_now_peer_info_t *peer, float temperature, float humidity);

#endif