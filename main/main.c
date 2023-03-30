/* Captive Portal Example

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <sys/param.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/inet.h"

#include "esp_http_server.h"
#include "dns_server.h"
#include "accel_read.h"

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN

extern const char root_start[] asm("_binary_plot_html_start");
extern const char root_end[] asm("_binary_plot_html_end");
extern const char js_start[] asm("_binary_uPlot_iife_min_js_start");
extern const char js_end[] asm("_binary_uPlot_iife_min_js_end");
extern const char css_start[] asm("_binary_uPlot_min_css_start");
extern const char css_end[] asm("_binary_uPlot_min_css_end");
extern const char myjs_start[] asm("_binary_liveplot_js_start");
extern const char myjs_end[] asm("_binary_liveplot_js_end");


static const char *TAG = "example";

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

// HTTP GET Handler
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const uint32_t root_len = root_end - root_start;
    ESP_LOGI(TAG, "Serve root");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, root_start, root_len);
    return ESP_OK;
}
static esp_err_t js_get_handler(httpd_req_t *req)
{
    const uint32_t len = js_end - js_start;
    ESP_LOGI(TAG, "Serve uPlot.js");
    httpd_resp_set_type(req, "text/javascript");
    httpd_resp_send(req, js_start, len);
    return ESP_OK;
}
static esp_err_t css_get_handler(httpd_req_t *req)
{
    const uint32_t len = css_end - css_start;
    ESP_LOGI(TAG, "Serve uPlot.css");
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, css_start, len);
    return ESP_OK;
}
static esp_err_t myjs_get_handler(httpd_req_t *req)
{
    const uint32_t len = myjs_end - myjs_start;
    ESP_LOGI(TAG, "Serve liveplot.js");
    httpd_resp_set_type(req, "text/javascript");
    httpd_resp_send(req, myjs_start, len);
    return ESP_OK;
}

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
    bool open;
};

#define CMAX 10
struct async_resp_arg clist[CMAX];
size_t ccount;

static esp_err_t data_get_handler(httpd_req_t *req)
{
    size_t i;
    for(i=0; i<ccount; ++i){
        if(!clist[i].open)
            break;
    }
    if(i < CMAX){
        if(i >= ccount)
            ccount = i+1;
        httpd_resp_set_type(req, HTTPD_TYPE_JSON);
        const char * data = "{'axes':['x','y','z'],'data':[]}\n";
        clist[i].hd = req->handle;
        clist[i].fd = httpd_req_to_sockfd(req);
        clist[i].open = 1;
        httpd_resp_send_chunk(req, data, 33);
    } else {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_send(req, "The ESP32 can only handle 10 connections", HTTPD_RESP_USE_STRLEN);
        ESP_LOGW("stream","Connection refused");
    }
    return ESP_OK;
}

void write_chunk_size(char * buf, size_t len){
    // Assume that len is less than 1500, due to MTU size limitations
    // Write three digits of hexadecimal. Covers numbers up to 4095
    len -= 7; // We're using the first five bytes to write the chunk size
    for(int i = 2; i >= 0; --i){
        char hex = len & 0xf;
        len >>= 4;
        if(hex > 9)
            hex += 'A'-10;
        else
            hex += '0';
        buf[i] = hex;
    }
    // Terminate with CRLF
    buf[3] = '\r';
    buf[4] = '\n';
}

void streaming_connection_task(void *pvParameters)
{
    const char * name = "stream";
    struct accel_t accel;
    QueueHandle_t queue = (QueueHandle_t) pvParameters;
    char s[1300];
    ccount = 0;
    while(1){
        size_t len = 5;
        len += snprintf(s+len, sizeof(s)-len, "{'axes':['x','y','z'],'data':[ ");
        while(xQueueReceive(queue,&accel,100/portTICK_PERIOD_MS)){
            len += snprintf(s+len,sizeof(s)-len,"[%d,%d,%d],",accel.x,accel.y,accel.z);
            if((sizeof(s)-len) < 30){
                break;
            }
        }
        memcpy(s+len-1, "]}\r\n", 4);
        len += 4;
        //ESP_LOGI(name, "JSON size: %d", len);
        write_chunk_size(s,len);
        for(int i = 0; i < ccount; ++i){
            if(!clist[i].open)
                continue;
            if(0xfffffffe == httpd_socket_send(clist[i].hd,clist[i].fd, s, len-1, 0)){
                ESP_LOGW(name, "Connection dropped");
                clist[i].open = 0;
            }
        }
    }
}

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};
static const httpd_uri_t js = {
    .uri = "/static/uPlot/dist/uPlot.iife.js",
    .method = HTTP_GET,
    .handler = js_get_handler
};
static const httpd_uri_t css = {
    .uri = "/static/uPlot/dist/uPlot.min.css",
    .method = HTTP_GET,
    .handler = css_get_handler
};

static const httpd_uri_t data = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = data_get_handler 
};
static const httpd_uri_t myjs = {
    .uri = "/static/liveplot.js",
    .method = HTTP_GET,
    .handler = myjs_get_handler
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
    //config.uri_match_fn = httpd_uri_match_wildcard;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &data);
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &js);
        httpd_register_uri_handler(server, &css);
        httpd_register_uri_handler(server, &myjs);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
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

    QueueHandle_t accel_queue = xQueueCreate(500, sizeof(struct accel_t));
    // Start the acceleration measurement
    xTaskCreate(accel_reader_task, "Accel Reader", 4096, accel_queue, 2, NULL);
    // Start the streaming task
    xTaskCreate(streaming_connection_task, "Data Stream", 4096, accel_queue, 2, NULL);

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

    // Start the server for the first time
    start_webserver();

    // Start the DNS server that will redirect all queries to the softAP IP
    start_dns_server();

}
