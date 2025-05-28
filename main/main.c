#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_http_server.h"

static const char *TAG = "AP_HTTP_SERVER";
static const char *BUTTON_CONFIG_TAG = "BUTTON_CONFIG";

/* An example web page served at the root URL */
static const char *HTML_PAGE =
    "<!DOCTYPE html>\n"
    "<html>\n"
    "<head>\n"
    "    <meta charset=\\\"UTF-8\\\">\n"
    "    <title>Button Configuration</title>\n"
    "</head>\n"
    "<body>\n"
    "    <h1>Configure Buttons</h1>\n"
    "\n"
    "    <form action=\\\"/api/config\\\" method=\\\"POST\\\" onsubmit=\\\"return onSubmitForm();\\\">\n"
    "        <label for=\\\"volumeBrand\\\">Volume buttons:</label>\n"
    "        <select id=\\\"volumeBrand\\\" name=\\\"volumeBrand\\\" onchange=\\\"onBrandChange('volumeBrand', 'volumeFrequency')\\\">\n"
    "            <option value=\\\"Host\\\">Host</option>\n"
    "            <option value=\\\"BrandA\\\">BrandA</option>\n"
    "            <option value=\\\"BrandB\\\">BrandB</option>\n"
    "            <option value=\\\"BrandC\\\">BrandC</option>\n"
    "        </select>\n"
    "        <select id=\\\"volumeFrequency\\\" name=\\\"volumeFrequency\\\" style=\\\"display: none;\\\">\n"
    "            <option value=\\\"1\\\">1</option><option value=\\\"2\\\">2</option><option value=\\\"3\\\">3</option>\n"
    "            <option value=\\\"4\\\">4</option><option value=\\\"5\\\">5</option><option value=\\\"6\\\">6</option>\n"
    "            <option value=\\\"7\\\">7</option><option value=\\\"8\\\">8</option><option value=\\\"9\\\">9</option>\n"
    "            <option value=\\\"10\\\">10</option>\n"
    "        </select>\n"
    "        <br><br>\n"
    "\n"
    "        <label for=\\\"powerBrand\\\">Power button:</label>\n"
    "        <select id=\\\"powerBrand\\\" name=\\\"powerBrand\\\" onchange=\\\"onBrandChange('powerBrand', 'powerFrequency')\\\">\n"
    "            <option value=\\\"Host\\\">Host</option>\n"
    "            <option value=\\\"BrandA\\\">BrandA</option>\n"
    "            <option value=\\\"BrandB\\\">BrandB</option>\n"
    "            <option value=\\\"BrandC\\\">BrandC</option>\n"
    "        </select>\n"
    "        <select id=\\\"powerFrequency\\\" name=\\\"powerFrequency\\\" style=\\\"display: none;\\\">\n"
    "            <option value=\\\"1\\\">1</option><option value=\\\"2\\\">2</option><option value=\\\"3\\\">3</option>\n"
    "            <option value=\\\"4\\\">4</option><option value=\\\"5\\\">5</option><option value=\\\"6\\\">6</option>\n"
    "            <option value=\\\"7\\\">7</option><option value=\\\"8\\\">8</option><option value=\\\"9\\\">9</option>\n"
    "            <option value=\\\"10\\\">10</option>\n"
    "        </select>\n"
    "        <br><br>\n"
    "\n"
    "        <label for=\\\"inputBrand\\\">Input button:</label>\n"
    "        <select id=\\\"inputBrand\\\" name=\\\"inputBrand\\\" onchange=\\\"onBrandChange('inputBrand', 'inputFrequency')\\\">\n"
    "            <option value=\\\"Host\\\">Host</option>\n"
    "            <option value=\\\"BrandA\\\">BrandA</option>\n"
    "            <option value=\\\"BrandB\\\">BrandB</option>\n"
    "            <option value=\\\"BrandC\\\">BrandC</option>\n"
    "        </select>\n"
    "        <select id=\\\"inputFrequency\\\" name=\\\"inputFrequency\\\" style=\\\"display: none;\\\">\n"
    "            <option value=\\\"1\\\">1</option><option value=\\\"2\\\">2</option><option value=\\\"3\\\">3</option>\n"
    "            <option value=\\\"4\\\">4</option><option value=\\\"5\\\">5</option><option value=\\\"6\\\">6</option>\n"
    "            <option value=\\\"7\\\">7</option><option value=\\\"8\\\">8</option><option value=\\\"9\\\">9</option>\n"
    "            <option value=\\\"10\\\">10</option>\n"
    "        </select>\n"
    "        <br><br>\n"
    "\n"
    "        <button type=\\\"submit\\\">Set</button>\n"
    "    </form>\n"
    "\n"
    "    <script>\n"
    "        function onBrandChange(brandSelectId, frequencySelectId) {\n"
    "            var brandSelect = document.getElementById(brandSelectId);\n"
    "            var frequencySelect = document.getElementById(frequencySelectId);\n"
    "            if (brandSelect.value === \\\"Host\\\") {\n"
    "                frequencySelect.style.display = \\\"none\\\";\n"
    "            } else {\n"
    "                frequencySelect.style.display = \\\"inline-block\\\";\n"
    "            }\n"
    "        }\n"
    "\n"
    "        function onSubmitForm() {\n"
    "            ['volumeBrand','powerBrand','inputBrand'].forEach(function(id) {\n"
    "                var brand = document.getElementById(id).value;\n"
    "                var freqSelectId = id.replace('Brand', 'Frequency');\n"
    "                if (brand === 'Host') {\n"
    "                    document.getElementById(freqSelectId).value = '';\n"
    "                }\n"
    "            });\n"
    "            return true;\n"
    "        }\n"
    "    </script>\n"
    "</body>\n"
    "</html>\n";



/* A simple handler that returns the HTML_PAGE */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_PAGE, strlen(HTML_PAGE));
    return ESP_OK;
}

static esp_err_t config_post_handler(httpd_req_t *req)
{
    // 1) Read request content
    char buf[200];
    int total_len = req->content_len;
    int cur_len = 0;
    int received = 0;

    if (total_len >= sizeof(buf)) {
        ESP_LOGE(TAG, "Content too large (%d bytes)", total_len);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Content too large");
        return ESP_FAIL;
    }

    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len - cur_len);
        if (received <= 0) {
            ESP_LOGE(TAG, "Failed to receive POST data");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read POST data");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[cur_len] = '\0'; // Null-terminate the data

    ESP_LOGI(TAG, "Received form data: %s", buf);

    // 2) Parse the form data (key-value pairs)
    char volumeBrand[32] = {0};
    char volumeFrequency[32] = {0};
    char powerBrand[32] = {0};
    char powerFrequency[32] = {0};
    char inputBrand[32] = {0};
    char inputFrequency[32] = {0};

    // Attempt to parse each field; if not present, it'll remain empty
    httpd_query_key_value(buf, "volumeBrand", volumeBrand, sizeof(volumeBrand));
    httpd_query_key_value(buf, "volumeFrequency", volumeFrequency, sizeof(volumeFrequency));
    httpd_query_key_value(buf, "powerBrand", powerBrand, sizeof(powerBrand));
    httpd_query_key_value(buf, "powerFrequency", powerFrequency, sizeof(powerFrequency));
    httpd_query_key_value(buf, "inputBrand", inputBrand, sizeof(inputBrand));
    httpd_query_key_value(buf, "inputFrequency", inputFrequency, sizeof(inputFrequency));

    ESP_LOGI(TAG, "Parsed values:");
    ESP_LOGI(TAG, "  Volume brand:     %s", volumeBrand);
    ESP_LOGI(TAG, "  Volume frequency: %s", volumeFrequency);
    ESP_LOGI(TAG, "  Power brand:      %s", powerBrand);
    ESP_LOGI(TAG, "  Power frequency:  %s", powerFrequency);
    ESP_LOGI(TAG, "  Input brand:      %s", inputBrand);
    ESP_LOGI(TAG, "  Input frequency:  %s", inputFrequency);

    // 3) Do something with these values:
    // For instance, store them in NVS, adjust IR transmitter settings, etc.

    // 4) Send a response back
    httpd_resp_sendstr(req, "Button configuration updated successfully!");

    return ESP_OK;
}


/* URI handler structure for GET / */
static httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static httpd_uri_t config_post_uri = {
    .uri       = "/api/config",
    .method    = HTTP_POST,
    .handler   = config_post_handler,
    .user_ctx  = NULL
};


/* Function to start the web server */
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Optionally change server port (default is 80)
    // config.server_port = 80;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &config_post_uri);
    } else {
        ESP_LOGE(TAG, "Error starting server!");
        return NULL;
    }
    return server;
}

/* Wi-Fi Event Handler */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "Wi-Fi AP started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station joined, AID=%u", event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station left, AID=%u", event->aid);
    }
}


void wifi_init_softap(void)
{
    /* Initialize TCP/IP and the event loop */
    esp_netif_init();
    esp_event_loop_create_default();

    /* Create default Wi-Fi AP. This gives us a network interface we can configure. */
    esp_netif_create_default_wifi_ap();

    /* Wi-Fi Config */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    /* Register event handler */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_register(WIFI_EVENT,
                                       ESP_EVENT_ANY_ID,
                                       &event_handler,
                                       NULL,
                                       &instance_any_id);

    /* Wi-Fi configuration for Access Point mode */
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32S3-AP",         // SSID of the AP
            .ssid_len = strlen("ESP32S3-AP"),
            .channel = 1,
            .password = "qwerty358",       // Password of the AP
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    // If password is empty, set authmode to WIFI_AUTH_OPEN
    if (strlen((char *)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    /* Set mode to AP and apply config */
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             wifi_config.ap.ssid, wifi_config.ap.password, wifi_config.ap.channel);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize and start Wi-Fi in AP mode */
    wifi_init_softap();

    /* Start HTTP server */
    start_webserver();
}
