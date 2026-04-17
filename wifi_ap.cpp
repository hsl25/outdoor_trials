#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/ip4_addr.h"
#include <stdio.h>

constexpr char AP_SSID[] = "Rover_Team_E";
constexpr char AP_PASS[] = "12345678";
constexpr int TCP_PORT = 5555;
bool wifi_ap_init() {

    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return false;
    }

    cyw43_arch_enable_ap_mode(
        AP_SSID,
        AP_PASS,
        CYW43_AUTH_WPA2_AES_PSK
    );

    printf("AP started\n");
    printf("SSID: %s\n", AP_SSID);
    printf("Password: %s\n", AP_PASS);

    return true;
}