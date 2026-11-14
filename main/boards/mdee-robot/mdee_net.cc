#include "mdee_net.h"
#include "config.h"
#include "mdee_util.h"

#include <esp_log.h>
#include <esp_netif.h>
#include <esp_sntp.h>
#include <esp_wifi.h>
#include <ctime>

#define TAG "MdeeNet"

namespace MdeeNet {

bool GetIp(char* buf, size_t len) {
    if (buf == nullptr || len == 0) return false;
    buf[0] = '\0';
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == nullptr) netif = esp_netif_get_default_netif();
    if (netif == nullptr) return false;
    esp_netif_ip_info_t info = {};
    if (esp_netif_get_ip_info(netif, &info) != ESP_OK || info.ip.addr == 0) return false;
    char tmp[64];
    const char* text = esp_ip4addr_ntoa(&info.ip, tmp, sizeof(tmp));
    if (text == nullptr) return false;
    MdeeCopy(buf, len, text);
    return buf[0] != '\0';
}

int Rssi() {
    wifi_ap_record_t ap = {};
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return -127;
    return ap.rssi;
}

void StartSntpOnce() {
    static bool started = false;
    if (started) return;
    char ip[64];
    if (!GetIp(ip, sizeof(ip))) return;
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    setenv("TZ", MDEE_TZ, 1);
    tzset();
    started = true;
    ESP_LOGI(TAG, "SNTP started (TZ=%s)", MDEE_TZ);
}

}  // namespace MdeeNet
