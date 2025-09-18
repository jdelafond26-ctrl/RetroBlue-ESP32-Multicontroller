#include "retroblue_settings.h"

RetroBlueSettings loaded_settings = {};

rb_err_t rb_settings_init(void) {
    nvs_handle_t my_handle;
    const char* TAG = "rb_settings_init";
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS Open failed.");
        return RB_FAIL;
    }

    size_t required_size = 0;
    err = nvs_get_blob(my_handle, "rb_settings", NULL, &required_size);
    
    if (required_size > 0 && err == ESP_OK) {
        err = nvs_get_blob(my_handle, "rb_settings", &loaded_settings, &required_size);
        if (err == ESP_OK && loaded_settings.magic_bytes == SETTINGS_MAGIC) {
            ESP_LOGI(TAG, "Settings loaded successfully");
            nvs_close(my_handle);
            return RB_OK;
        }
    }

    nvs_close(my_handle);
    return rb_settings_default();
}

rb_err_t rb_settings_saveall(void) {
    const char* TAG = "rb_settings_saveall";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS storage failed to open.");
        return RB_FAIL;
    }

    nvs_set_blob(my_handle, "rb_settings", &loaded_settings, sizeof(loaded_settings));
    nvs_commit(my_handle);
    nvs_close(my_handle);
    return RB_OK;
}

rb_err_t rb_settings_default(void) {
    const char* TAG = "rb_settings_default";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS storage.");
        return RB_FAIL;
    }

    // Configuration Switch Pro Controller uniquement
    loaded_settings.magic_bytes = SETTINGS_MAGIC;

    // Adresse BT
    for(int i = 0; i < 6; i++) {
        switch(i) {
            case 0: loaded_settings.ns_client_bt_address[i] = 0x98; break;
            case 1: loaded_settings.ns_client_bt_address[i] = 0x41; break;
            case 2: loaded_settings.ns_client_bt_address[i] = 0x5C; break;
            default: loaded_settings.ns_client_bt_address[i] = esp_random() % 255; break;
        }
    }
    
    memset(loaded_settings.ns_host_bt_address, 0, 6);
    loaded_settings.ns_controller_paired = false;

    // Couleur noire
    loaded_settings.color_r = 0x00;
    loaded_settings.color_g = 0x00;
    loaded_settings.color_b = 0x00;

    // Calibration par défaut
    loaded_settings.sx_min = 0xFA;
    loaded_settings.sx_center = 0x740;
    loaded_settings.sx_max = 0xF47;
    loaded_settings.sy_min = 0xFA;
    loaded_settings.sy_center = 0x740;
    loaded_settings.sy_max = 0xF47;

    nvs_set_blob(my_handle, "rb_settings", &loaded_settings, sizeof(loaded_settings));
    nvs_commit(my_handle);
    nvs_close(my_handle);
    return RB_OK;
}

// Fonction pour sauvegarder l'appairage Switch
// rb_err_t rbc_core_savepairing(uint8_t *host_addr) {
//     const char *TAG = "rbc_core_savepairing";

//     if (host_addr == NULL) {
//         ESP_LOGE(TAG, "Host address is blank.");
//         return RB_FAIL;
//     }

//     ESP_LOGI(TAG, "Pairing to Nintendo Switch.");
//     memcpy(loaded_settings.ns_host_bt_address, host_addr, sizeof(loaded_settings.ns_host_bt_address));
//     loaded_settings.ns_controller_paired = true;

//     if (rb_settings_saveall() == RB_OK) {
//         ESP_LOGI(TAG, "Pairing saved successfully.");
//         return RB_OK;
//     } else {
//         ESP_LOGE(TAG, "Failed to save pairing.");
//         return RB_FAIL;
//     }
// }