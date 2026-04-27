/* Captive Portal Example

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <sys/param.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/inet.h"

#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "dns_server.h"
#include "accel_read.h"
#include "cJSON.h"

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN

static const char *TAG = "example";
static httpd_handle_t s_server;

static bool ends_with(const char *str, const char *suffix)
{
    size_t str_len = strlen(str);
    size_t suffix_len = strlen(suffix);
    if (str_len < suffix_len) {
        return false;
    }
    return strncmp(str + str_len - suffix_len, suffix, suffix_len) == 0;
}

static const char *content_type_for_uri(const char *uri)
{
    if (strcmp(uri, "/") == 0 || ends_with(uri, ".html")) {
        return "text/html";
    }
    if (ends_with(uri, ".js")) {
        return "application/javascript";
    }
    if (ends_with(uri, ".css")) {
        return "text/css";
    }
    return "application/octet-stream";
}

static esp_err_t send_gzip_file(httpd_req_t *req, const char *file_path, const char *content_type)
{
    FILE *file = fopen(file_path, "rb");
    if (file == NULL) {
        ESP_LOGW(TAG, "File not found: %s", file_path);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, content_type);
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    char chunk[1024];
    size_t read_size;
    do {
        read_size = fread(chunk, 1, sizeof(chunk), file);
        if (read_size > 0) {
            if (httpd_resp_send_chunk(req, chunk, read_size) != ESP_OK) {
                fclose(file);
                httpd_resp_sendstr_chunk(req, NULL);
                return ESP_FAIL;
            }
        }
    } while (read_size > 0);

    fclose(file);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);
    ESP_LOGI(TAG, "Set up softAP with IP: %s", ip_addr);

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:'%s' password:'%s'",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    return send_gzip_file(req, "/spiffs/plot.html.gz", "text/html");
}

static esp_err_t static_get_handler(httpd_req_t *req)
{
    const char *prefix = "/static";
    const size_t prefix_len = strlen(prefix);
    const char *uri = req->uri;

    if (strncmp(uri, prefix, prefix_len) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Invalid path");
        return ESP_FAIL;
    }

    const char *relative_path = uri + prefix_len;
    if (relative_path[0] == '\0') {
        relative_path = "/plot.html";
    }

    char file_path[320];
    int written = snprintf(file_path, sizeof(file_path), "/spiffs%s.gz", relative_path);
    if (written < 0 || written >= (int)sizeof(file_path)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Path too long");
        return ESP_FAIL;
    }

    return send_gzip_file(req, file_path, content_type_for_uri(uri));
}

static esp_err_t header_get_handler(httpd_req_t *req)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"scales\":{\"x\":{\"time\":false}},"
        "\"series\":[{\"label\":\"t\"},{\"label\":\"x\"},{\"label\":\"y\"},{\"label\":\"z\"}],"
        "\"sample_rate\":%"PRIu32","
        "\"scale_factor\":%.6f,"
        "\"dlpf_cfg\":%u,"
        "\"afs_sel\":%u}",
        accel_get_sample_rate(),
        accel_get_scale_factor(),
        (unsigned)accel_get_dlpf_cfg(),
        (unsigned)accel_get_afs_sel());
    httpd_resp_set_type(req, HTTPD_TYPE_JSON);
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t settings_post_handler(httpd_req_t *req)
{
    char body[128];
    int ret = httpd_req_recv(req, body, sizeof(body) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[ret] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *j_dlpf = cJSON_GetObjectItem(root, "dlpf_cfg");
    cJSON *j_afs  = cJSON_GetObjectItem(root, "afs_sel");

    if (!cJSON_IsNumber(j_dlpf) || !cJSON_IsNumber(j_afs)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing fields");
        return ESP_FAIL;
    }

    int dlpf_cfg = (int)cJSON_GetNumberValue(j_dlpf);
    int afs_sel  = (int)cJSON_GetNumberValue(j_afs);
    cJSON_Delete(root);

    if (dlpf_cfg < 0 || dlpf_cfg > 6 || afs_sel < 0 || afs_sel > 3) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Value out of range");
        return ESP_FAIL;
    }

    accel_set_config((uint8_t)dlpf_cfg, (uint8_t)afs_sel);
    httpd_resp_set_type(req, HTTPD_TYPE_JSON);
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t stream_ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake done for %s", req->uri);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = NULL,
        .len = 0,
    };

    esp_err_t err = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (err != ESP_OK) {
        return err;
    }

    if (ws_pkt.len > 0) {
        ws_pkt.payload = calloc(1, ws_pkt.len + 1);
        if (ws_pkt.payload == NULL) {
            return ESP_ERR_NO_MEM;
        }
        err = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        free(ws_pkt.payload);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

static void ws_stream_task(void *arg)
{
    static uint8_t payload[1024];
    size_t last_sent_idx = accel_get_latest_idx();

    while (1) {
        if (s_server != NULL) {
            int client_fds[CONFIG_LWIP_MAX_SOCKETS];
            size_t client_count = CONFIG_LWIP_MAX_SOCKETS;

            if (httpd_get_client_list(s_server, &client_count, client_fds) == ESP_OK && client_count > 0) {
                size_t payload_len = accel_copy_new_binary(last_sent_idx, payload, sizeof(payload));
                if (payload_len > 0) {
                    size_t n_sent = (payload_len - 6) / 6;
                    last_sent_idx = (last_sent_idx + n_sent) % 2000;
                    httpd_ws_frame_t ws_pkt = {
                        .final = true,
                        .fragmented = false,
                        .type = HTTPD_WS_TYPE_BINARY,
                        .payload = payload,
                        .len = payload_len,
                    };
                    for (size_t i = 0; i < client_count; i++) {
                        if (httpd_ws_get_fd_info(s_server, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                            httpd_ws_send_frame_async(s_server, client_fds[i], &ws_pkt);
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 8,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0;
    size_t used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS total=%u used=%u", (unsigned int)total, (unsigned int)used);
    }

    return ret;
}

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};
static const httpd_uri_t static_files = {
    .uri = "/static/*",
    .method = HTTP_GET,
    .handler = static_get_handler
};

static const httpd_uri_t header = {
    .uri = "/header",
    .method = HTTP_GET,
    .handler = header_get_handler
};
static const httpd_uri_t settings = {
    .uri = "/settings",
    .method = HTTP_POST,
    .handler = settings_post_handler
};

static const httpd_uri_t stream = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_ws_handler,
    .is_websocket = true,
    .handle_ws_control_frames = true
};

// HTTP Error (404) Handler - Redirects all requests to the root page
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // Set status
    httpd_resp_set_status(req, "302 Temporary Redirect");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    ESP_LOGI(TAG, "Redirecting to root");
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 13;
    config.lru_purge_enable = true;
    config.uri_match_fn = httpd_uri_match_wildcard;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &header);
        httpd_register_uri_handler(server, &settings);
        httpd_register_uri_handler(server, &stream);
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &static_files);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
        s_server = server;
    }
    return server;
}

void app_main(void)
{
    /*
        Turn of warnings from HTTP server as redirecting traffic will yield
        lots of invalid requests
    */
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd_parse", ESP_LOG_ERROR);

    // Start the acceleration measurement
    xTaskCreate(accel_reader_task, "Accel Reader", 4096, NULL, 2, NULL);

    // Initialize networking stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop needed by the  main app
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS needed by Wi-Fi
    ESP_ERROR_CHECK(nvs_flash_init());

    // Initialize Wi-Fi including netif with default config
    esp_netif_create_default_wifi_ap();

    // Initialise ESP32 in SoftAP mode
    wifi_init_softap();

    ESP_ERROR_CHECK(init_spiffs());

    // Start the server for the first time
    start_webserver();

    xTaskCreate(ws_stream_task, "WS Stream", 6144, NULL, 2, NULL);

    // Start the DNS server that will redirect all queries to the softAP IP
    start_dns_server();
}
