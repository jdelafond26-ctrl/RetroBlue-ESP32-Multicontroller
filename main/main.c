#include "main.h"

// ADC channels
#define ADC_STICK_LX ADC1_CHANNEL_0 // VP
#define ADC_STICK_LY ADC1_CHANNEL_3 // VN
#define ADC_STICK_RX ADC1_CHANNEL_6 // D34
#define ADC_STICK_RY ADC1_CHANNEL_7 // D35

// GPIO Boutons
#define GPIO_BTN_DU GPIO_NUM_25     // D25
#define GPIO_BTN_DD GPIO_NUM_26     // D26
#define GPIO_BTN_DL GPIO_NUM_27     // D27
#define GPIO_BTN_DR GPIO_NUM_14     // D14
#define GPIO_BTN_SELECT GPIO_NUM_12 // D12 (-)
#define GPIO_BTN_START GPIO_NUM_13  // D13 (+)
#define GPIO_BTN_L GPIO_NUM_23      // D23
#define GPIO_BTN_R GPIO_NUM_22      // D22
#define GPIO_BTN_ZL GPIO_NUM_32     // D32
#define GPIO_BTN_ZR GPIO_NUM_33     // D33
#define GPIO_BTN_STICKL GPIO_NUM_3  // RX0
#define GPIO_BTN_STICKR GPIO_NUM_21 // D21
#define GPIO_BTN_B GPIO_NUM_19      // D19
#define GPIO_BTN_A GPIO_NUM_18      // D18
#define GPIO_BTN_Y GPIO_NUM_5       // D5
#define GPIO_BTN_X GPIO_NUM_17      // TX2
#define GPIO_BTN_HOME GPIO_NUM_16   // RX2
#define GPIO_BTN_CAPTURE GPIO_NUM_4 // D4

// Calibration
#define GPIO_BTN_CALIBRATE GPIO_NUM_15  // D15
#define GPIO_LED_SYNC GPIO_NUM_1        // D1
#define GPIO_LED_CALIBRATE_L GPIO_NUM_0 // D0
#define GPIO_LED_CALIBRATE_R GPIO_NUM_2 // D2

#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_LED_SYNC) | (1ULL << GPIO_LED_CALIBRATE_L) | (1ULL << GPIO_LED_CALIBRATE_R))
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_BTN_CALIBRATE) | (1ULL << GPIO_BTN_DU) | (1ULL << GPIO_BTN_DD) | (1ULL << GPIO_BTN_DL) | (1ULL << GPIO_BTN_DR) | (1ULL << GPIO_BTN_STICKL) | (1ULL << GPIO_BTN_SELECT) | (1ULL << GPIO_BTN_L) | (1ULL << GPIO_BTN_ZL) | (1ULL << GPIO_BTN_STICKR) | (1ULL << GPIO_BTN_R) | (1ULL << GPIO_BTN_ZR) | (1ULL << GPIO_BTN_A) | (1ULL << GPIO_BTN_B) | (1ULL << GPIO_BTN_X) | (1ULL << GPIO_BTN_Y) | (1ULL << GPIO_BTN_START) | (1ULL << GPIO_BTN_HOME) | (1ULL << GPIO_BTN_CAPTURE))

#define CAL_MARGIN 50 // marge pour éviter min/max trop serrés

// Variables
uint32_t regread = 0;
const uint16_t DEADZONE = 80;
uint8_t current_battery_level = 0x8;

// Calibration des sticks
typedef struct
{
    uint16_t lx_min, lx_center, lx_max;
    uint16_t ly_min, ly_center, ly_max;
    uint16_t rx_min, rx_center, rx_max;
    uint16_t ry_min, ry_center, ry_max;
    bool calibrated;
} stick_calibration_t;

stick_calibration_t stick_cal = {0};
bool calibration_in_progress = false;

// Fonctions utilitaires
bool getbit(uint32_t bytes, uint8_t bit)
{
    return !((bytes >> bit) & 0x1);
}

uint16_t apply_deadzone(uint16_t raw, uint16_t center)
{
    int32_t diff = (int32_t)raw - (int32_t)center;
    if (abs(diff) < DEADZONE)
        return center;
    return raw;
}

// Fonction pour mettre à jour le niveau de batterie dans les données Switch
void update_switch_battery()
{
    extern ns_controller_data_s ns_controller_data; // Référence à la structure Switch
    ns_controller_data.battery_level_full = current_battery_level;
}

// Fonctions calibration améliorées
void save_stick_calibration() {
    const char* TAG = "save_stick_cal";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("calibration", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing");
        return;
    }

    stick_cal.calibrated = true;

    err = nvs_set_blob(my_handle, "stick_cal", &stick_cal, sizeof(stick_calibration_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set blob in NVS");
        nvs_close(my_handle);
        return;
    }

    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS");
    } else {
        ESP_LOGI(TAG, "Calibration saved successfully: calibrated=%d", stick_cal.calibrated);
    }

    nvs_close(my_handle);
}


void load_stick_calibration() {
    const char* TAG = "load_stick_cal";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("calibration", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No calibration NVS found, using defaults");
        stick_cal.calibrated = false;
        return;
    }

    size_t required_size = sizeof(stick_calibration_t);
    err = nvs_get_blob(my_handle, "stick_cal", &stick_cal, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read calibration, using defaults");
        stick_cal.calibrated = false;
    } else {
        ESP_LOGI(TAG, "Calibration loaded: calibrated=%d", stick_cal.calibrated);
    }

    nvs_close(my_handle);
}

void blink_led(uint8_t gpio, uint32_t duration_ms, uint32_t interval_ms)
{
    uint32_t start_time = xTaskGetTickCount();
    bool led_state = false;
    while ((xTaskGetTickCount() - start_time) < (duration_ms / portTICK_PERIOD_MS))
    {
        led_state = !led_state;
        gpio_set_level(gpio, led_state);
        vTaskDelay(interval_ms / portTICK_PERIOD_MS);
    }
    gpio_set_level(gpio, 0);
}

void blink_leds(uint32_t duration_ms, uint32_t interval_ms)
{
    uint32_t start_time = xTaskGetTickCount();
    bool led_state = false;
    while ((xTaskGetTickCount() - start_time) < (duration_ms / portTICK_PERIOD_MS))
    {
        led_state = !led_state;
        gpio_set_level(GPIO_LED_CALIBRATE_L, led_state);
        gpio_set_level(GPIO_LED_CALIBRATE_R, led_state);
        vTaskDelay(interval_ms / portTICK_PERIOD_MS);
    }
    gpio_set_level(GPIO_LED_CALIBRATE_L, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 0);
}

// Fonction améliorée pour mesurer le centre du stick
void measure_stick_center(uint8_t adc_x, uint8_t adc_y, uint16_t *center_x, uint16_t *center_y, uint8_t led_gpio) {
    const uint16_t STABILITY_THRESHOLD = 25;
    const int NUM_SAMPLES = 100;
    const uint32_t TIMEOUT_MS = 15000;
    const int CONSECUTIVE_STABLE = 8;

    ESP_LOGI("CALIB", "=== CENTER MEASUREMENT ===");
    ESP_LOGI("CALIB", "Position stick at CENTER and keep it steady!");
    ESP_LOGI("CALIB", "Starting measurement in 3 seconds...");
    
    gpio_set_level(led_gpio, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_LOGI("CALIB", "Measuring center - DO NOT MOVE STICK!");
    uint32_t start_time = xTaskGetTickCount();

    uint32_t sum_x = 0, sum_y = 0;
    uint16_t last_x = 0, last_y = 0;
    int valid_samples = 0;
    int stable_count = 0;
    bool first_read = true;

    // Buffer pour analyser la stabilité
    uint16_t readings_x[10] = {0};
    uint16_t readings_y[10] = {0};
    int buffer_idx = 0;
    
    while (valid_samples < NUM_SAMPLES) {
        if ((xTaskGetTickCount() - start_time) > (TIMEOUT_MS / portTICK_PERIOD_MS)) {
            ESP_LOGW("CALIB", "Timeout during center measurement");
            break;
        }

        uint16_t val_x = adc1_get_raw(adc_x);
        uint16_t val_y = adc1_get_raw(adc_y);
        
        // Stocker dans le buffer circulaire
        readings_x[buffer_idx % 10] = val_x;
        readings_y[buffer_idx % 10] = val_y;
        buffer_idx++;

        if (first_read) {
            last_x = val_x;
            last_y = val_y;
            first_read = false;
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        // Calculer la variance sur les dernières lectures
        if (buffer_idx >= 10) {
            uint32_t avg_x = 0, avg_y = 0;
            for (int i = 0; i < 10; i++) {
                avg_x += readings_x[i];
                avg_y += readings_y[i];
            }
            avg_x /= 10;
            avg_y /= 10;
            
            uint32_t variance_x = 0, variance_y = 0;
            for (int i = 0; i < 10; i++) {
                variance_x += abs(readings_x[i] - avg_x);
                variance_y += abs(readings_y[i] - avg_y);
            }
            variance_x /= 10;
            variance_y /= 10;
            
            // Si stable sur les 10 dernières lectures
            if (variance_x <= STABILITY_THRESHOLD && variance_y <= STABILITY_THRESHOLD) {
                stable_count++;
                
                if (stable_count >= CONSECUTIVE_STABLE) {
                    sum_x += val_x;
                    sum_y += val_y;
                    valid_samples++;
                    stable_count = 0;
                    
                    // Feedback visuel : clignotement rapide quand on capture
                    gpio_set_level(led_gpio, valid_samples % 2);
                    
                    // Log périodique
                    if (valid_samples % 10 == 0) {
                        ESP_LOGI("CALIB", "Captured %d/%d samples, current: (%d,%d)", 
                                 valid_samples, NUM_SAMPLES, val_x, val_y);
                    }
                }
            } else {
                stable_count = 0;
                // LED fixe si pas stable
                gpio_set_level(led_gpio, 1);
            }
        }

        last_x = val_x;
        last_y = val_y;
        vTaskDelay(30 / portTICK_PERIOD_MS);
    }

    if (valid_samples > 0) {
        *center_x = sum_x / valid_samples;
        *center_y = sum_y / valid_samples;
    } else {
        ESP_LOGW("CALIB", "No stable samples, using current reading");
        *center_x = adc1_get_raw(adc_x);
        *center_y = adc1_get_raw(adc_y);
    }

    gpio_set_level(led_gpio, 0);
    ESP_LOGI("CALIB", "Center captured: X=%d, Y=%d (from %d stable samples)", 
             *center_x, *center_y, valid_samples);
    
    // Validation immédiate
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint16_t verify_x = adc1_get_raw(adc_x);
    uint16_t verify_y = adc1_get_raw(adc_y);
    
    ESP_LOGI("CALIB", "Verification reading: X=%d, Y=%d", verify_x, verify_y);
    ESP_LOGI("CALIB", "Difference from center: X=%d, Y=%d", 
             abs(verify_x - *center_x), abs(verify_y - *center_y));
    
    if (abs(verify_x - *center_x) > 50 || abs(verify_y - *center_y) > 50) {
        ESP_LOGW("CALIB", "WARNING: Stick may have moved during measurement!");
    }
}

// Fonction pour mesurer min/max d’un stick pendant qu’on fait des cercles
void measure_stick_circles(uint8_t adc_x, uint8_t adc_y, uint16_t *min_x, uint16_t *max_x, uint16_t *min_y, uint16_t *max_y, uint8_t led_gpio)
{
    *min_x = *max_x = adc1_get_raw(adc_x);
    *min_y = *max_y = adc1_get_raw(adc_y);
    uint32_t start_time = xTaskGetTickCount();
    bool led_state = false;

    ESP_LOGI("CALIB", "Move stick in full circles for 8 seconds - START NOW!");

    while ((xTaskGetTickCount() - start_time) < (8000 / portTICK_PERIOD_MS))
    {
        if ((xTaskGetTickCount() - start_time) % (200 / portTICK_PERIOD_MS) == 0)
        {
            led_state = !led_state;
            gpio_set_level(led_gpio, led_state);
        }
        uint16_t val_x = adc1_get_raw(adc_x);
        uint16_t val_y = adc1_get_raw(adc_y);

        if (val_x < *min_x) *min_x = val_x;
        if (val_x > *max_x) *max_x = val_x;
        if (val_y < *min_y) *min_y = val_y;
        if (val_y > *max_y) *max_y = val_y;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    gpio_set_level(led_gpio, 0);
    ESP_LOGI("CALIB", "Circles done - X: %d to %d, Y: %d to %d", *min_x, *max_x, *min_y, *max_y);
}


uint16_t apply_calibration(uint16_t raw, uint16_t min_val, uint16_t center, uint16_t max_val)
{
    if (!stick_cal.calibrated) {
        return apply_deadzone(raw, 2048);
    }

    // Protection contre des valeurs incohérentes
    if (min_val >= center || center >= max_val) {
        ESP_LOGW("CALIB", "Invalid calibration values, using default");
        return apply_deadzone(raw, 2048);
    }

    int32_t diff = (int32_t)raw - (int32_t)center;
    
    // Zone morte symétrique autour du centre
    if (abs(diff) < DEADZONE) {
        return 2048; // Toujours retourner exactement le centre Switch
    }

    if (raw < center) {
        // Côté négatif : mapper [min_val, center-deadzone] vers [0, 2048-1]
        if (raw <= min_val) return 0;
        
        uint32_t usable_range = center - min_val - DEADZONE;
        uint32_t position = center - raw - DEADZONE;
        
        if (usable_range > 0) {
            return 2048 - 1 - (uint16_t)((position * 2047) / usable_range);
        } else {
            return 2048;
        }
    } else {
        // Côté positif : mapper [center+deadzone, max_val] vers [2048+1, 4095]
        if (raw >= max_val) return 4095;
        
        uint32_t usable_range = max_val - center - DEADZONE;
        uint32_t position = raw - center - DEADZONE;
        
        if (usable_range > 0) {
            return 2048 + 1 + (uint16_t)((position * 2047) / usable_range);
        } else {
            return 2048;
        }
    }
}

// Fonction de test de calibration avec logs détaillés
void test_calibration_detailed() {
    if (!stick_cal.calibrated) {
        ESP_LOGW("TEST", "No calibration data available");
        return;
    }

    ESP_LOGI("TEST", "=== CALIBRATION TEST ===");
    ESP_LOGI("TEST", "Left stick calibration: min=%d, center=%d, max=%d", 
             stick_cal.lx_min, stick_cal.lx_center, stick_cal.lx_max);
    ESP_LOGI("TEST", "Testing calibration for 15 seconds - move left stick...");
    
    uint32_t start_time = xTaskGetTickCount();
    uint32_t last_log = 0;
    
    while ((xTaskGetTickCount() - start_time) < (15000 / portTICK_PERIOD_MS)) {
        uint32_t now = xTaskGetTickCount();
        
        if (now - last_log > (500 / portTICK_PERIOD_MS)) {
            uint16_t raw_lx = adc1_get_raw(ADC_STICK_LX);
            uint16_t raw_ly = adc1_get_raw(ADC_STICK_LY);
            
            uint16_t cal_lx = apply_calibration(raw_lx, stick_cal.lx_min, stick_cal.lx_center, stick_cal.lx_max);
            uint16_t cal_ly = apply_calibration(raw_ly, stick_cal.ly_min, stick_cal.ly_center, stick_cal.ly_max);
            
            // Calculer la position relative au centre
            int32_t offset_x = (int32_t)cal_lx - 2048;
            int32_t offset_y = (int32_t)cal_ly - 2048;
            
            ESP_LOGI("TEST", "RAW(%d,%d) -> CAL(%d,%d) -> OFFSET(%d,%d)", 
                     raw_lx, raw_ly, cal_lx, cal_ly, offset_x, offset_y);
            
            last_log = now;
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI("TEST", "Test finished");
}

// Fonction de calibration réorganisée : centre d'abord !
void calibrate_sticks() {
    ESP_LOGI("CALIB", "=== ADVANCED STICK CALIBRATION ===");
    ESP_LOGI("CALIB", "This calibration will be more precise.");
    ESP_LOGI("CALIB", "Follow instructions carefully for best results.");
    
    gpio_set_level(GPIO_LED_CALIBRATE_L, 1);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    calibration_in_progress = true;

    // === ÉTAPE 1: CENTRES PRÉCIS ===
    ESP_LOGI("CALIB", "STEP 1/4: LEFT stick center");
    measure_stick_center(ADC_STICK_LX, ADC_STICK_LY, 
                                   &stick_cal.lx_center, &stick_cal.ly_center, 
                                   GPIO_LED_CALIBRATE_L);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI("CALIB", "STEP 2/4: RIGHT stick center");
    measure_stick_center(ADC_STICK_RX, ADC_STICK_RY, 
                                   &stick_cal.rx_center, &stick_cal.ry_center, 
                                   GPIO_LED_CALIBRATE_R);

    blink_leds(3000, 150);

    // === ÉTAPE 2: PLAGES COMPLÈTES ===
    ESP_LOGI("CALIB", "STEP 3/4: LEFT stick full range - move in complete circles");
    measure_stick_circles(ADC_STICK_LX, ADC_STICK_LY, 
                         &stick_cal.lx_min, &stick_cal.lx_max, 
                         &stick_cal.ly_min, &stick_cal.ly_max, 
                         GPIO_LED_CALIBRATE_L);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI("CALIB", "STEP 4/4: RIGHT stick full range - move in complete circles");
    measure_stick_circles(ADC_STICK_RX, ADC_STICK_RY, 
                         &stick_cal.rx_min, &stick_cal.rx_max, 
                         &stick_cal.ry_min, &stick_cal.ry_max, 
                         GPIO_LED_CALIBRATE_R);

    // === VALIDATION COMPLÈTE ===
    ESP_LOGI("CALIB", "=== VALIDATING CALIBRATION DATA ===");
    
    bool calibration_valid = true;
    
    // Vérification des plages
    uint16_t lx_range = stick_cal.lx_max - stick_cal.lx_min;
    uint16_t ly_range = stick_cal.ly_max - stick_cal.ly_min;
    uint16_t rx_range = stick_cal.rx_max - stick_cal.rx_min;
    uint16_t ry_range = stick_cal.ry_max - stick_cal.ry_min;
    
    ESP_LOGI("CALIB", "Ranges - LX:%d, LY:%d, RX:%d, RY:%d", 
             lx_range, ly_range, rx_range, ry_range);
    
    if (lx_range < 1000 || ly_range < 1000 || rx_range < 1000 || ry_range < 1000) {
        ESP_LOGW("CALIB", "WARNING: Some ranges seem too small (< 1000)");
        calibration_valid = false;
    }
    
    // Vérification des centres
    if (stick_cal.lx_center <= stick_cal.lx_min || stick_cal.lx_center >= stick_cal.lx_max) {
        ESP_LOGW("CALIB", "Left X center outside range, adjusting");
        stick_cal.lx_center = (stick_cal.lx_min + stick_cal.lx_max) / 2;
    }
    if (stick_cal.ly_center <= stick_cal.ly_min || stick_cal.ly_center >= stick_cal.ly_max) {
        ESP_LOGW("CALIB", "Left Y center outside range, adjusting");
        stick_cal.ly_center = (stick_cal.ly_min + stick_cal.ly_max) / 2;
    }
    if (stick_cal.rx_center <= stick_cal.rx_min || stick_cal.rx_center >= stick_cal.rx_max) {
        ESP_LOGW("CALIB", "Right X center outside range, adjusting");
        stick_cal.rx_center = (stick_cal.rx_min + stick_cal.rx_max) / 2;
    }
    if (stick_cal.ry_center <= stick_cal.ry_min || stick_cal.ry_center >= stick_cal.ry_max) {
        ESP_LOGW("CALIB", "Right Y center outside range, adjusting");
        stick_cal.ry_center = (stick_cal.ry_min + stick_cal.ry_max) / 2;
    }

    // Affichage des résultats finaux
    ESP_LOGI("CALIB", "=== FINAL CALIBRATION RESULTS ===");
    ESP_LOGI("CALIB", "Left stick  - X: %d - %d - %d (range: %d)", 
             stick_cal.lx_min, stick_cal.lx_center, stick_cal.lx_max, lx_range);
    ESP_LOGI("CALIB", "Left stick  - Y: %d - %d - %d (range: %d)", 
             stick_cal.ly_min, stick_cal.ly_center, stick_cal.ly_max, ly_range);
    ESP_LOGI("CALIB", "Right stick - X: %d - %d - %d (range: %d)", 
             stick_cal.rx_min, stick_cal.rx_center, stick_cal.rx_max, rx_range);
    ESP_LOGI("CALIB", "Right stick - Y: %d - %d - %d (range: %d)", 
             stick_cal.ry_min, stick_cal.ry_center, stick_cal.ry_max, ry_range);

    // Animation de fin selon le résultat
    if (calibration_valid) {
        ESP_LOGI("CALIB", "Calibration completed successfully!");
        // Clignotement lent = succès
        for (int i = 0; i < 6; i++) {
            gpio_set_level(GPIO_LED_CALIBRATE_L, i % 2);
            gpio_set_level(GPIO_LED_CALIBRATE_R, i % 2);
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGW("CALIB", "Calibration completed with warnings!");
        // Clignotement rapide = avertissement
        for (int i = 0; i < 12; i++) {
            gpio_set_level(GPIO_LED_CALIBRATE_L, i % 2);
            gpio_set_level(GPIO_LED_CALIBRATE_R, i % 2);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

    gpio_set_level(GPIO_LED_CALIBRATE_L, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 0);

    stick_cal.calibrated = true;
    calibration_in_progress = false;
    save_stick_calibration();
    
    // Test automatique après calibration
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI("CALIB", "Starting automatic test...");
    test_calibration_detailed();
}

// SUPPRESSION DE LA REDONDANCE : On garde seulement dans button_task
void button_task()
{
    regread = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_SEL;

    // Gestion calibration (3 secondes) - UNIQUE ENDROIT
    static uint32_t calibrate_pressed_time = 0;
    if (!gpio_get_level(GPIO_BTN_CALIBRATE) && !calibration_in_progress)
    {
        if (calibrate_pressed_time == 0)
        {
            calibrate_pressed_time = xTaskGetTickCount();
            ESP_LOGI("CALIB", "Calibration button pressed - hold for 3s...");
        }
        else if ((xTaskGetTickCount() - calibrate_pressed_time) > (3000 / portTICK_PERIOD_MS))
        {
            ESP_LOGI("CALIB", "3 seconds reached - starting calibration");
            calibrate_sticks();
            calibrate_pressed_time = 0;
        }
    }
    else
    {
        if (calibrate_pressed_time != 0) {
            ESP_LOGI("CALIB", "Calibration button released");
        }
        calibrate_pressed_time = 0;
    }

    if (calibration_in_progress) return;

    // Boutons Switch Pro Controller
    g_button_data.d_up = getbit(regread, GPIO_BTN_DU);
    g_button_data.d_down = getbit(regread, GPIO_BTN_DD);
    g_button_data.d_left = getbit(regread, GPIO_BTN_DL);
    g_button_data.d_right = getbit(regread, GPIO_BTN_DR);

    g_button_data.b_right = getbit(regread, GPIO_BTN_A); // A
    g_button_data.b_down = getbit(regread, GPIO_BTN_B);  // B
    g_button_data.b_up = getbit(regread, GPIO_BTN_X);    // X
    g_button_data.b_left = getbit(regread, GPIO_BTN_Y);  // Y

    g_button_data.t_l = getbit(regread, GPIO_BTN_L);
    g_button_data.t_r = getbit(regread, GPIO_BTN_R);
    g_button_data.t_zl = !gpio_get_level(GPIO_BTN_ZL);
    g_button_data.t_zr = !gpio_get_level(GPIO_BTN_ZR);

    g_button_data.b_select = getbit(regread, GPIO_BTN_SELECT);
    g_button_data.b_start = getbit(regread, GPIO_BTN_START);
    g_button_data.b_capture = getbit(regread, GPIO_BTN_CAPTURE);
    g_button_data.b_home = getbit(regread, GPIO_BTN_HOME);

    g_button_data.sb_left = getbit(regread, GPIO_BTN_STICKL);
    g_button_data.sb_right = getbit(regread, GPIO_BTN_STICKR);
}

void stick_task()
{
    static uint32_t last_log_tick = 0;

    if (calibration_in_progress) return;

    uint16_t raw_lsx = adc1_get_raw(ADC_STICK_LX);
    uint16_t raw_lsy = adc1_get_raw(ADC_STICK_LY);
    uint16_t raw_rsx = adc1_get_raw(ADC_STICK_RX);
    uint16_t raw_rsy = adc1_get_raw(ADC_STICK_RY);

    uint16_t cal_lsx, cal_lsy, cal_rsx, cal_rsy;

    // if (stick_cal.calibrated) {
    //     cal_lsx = apply_calibration(raw_lsx, stick_cal.lx_min, stick_cal.lx_center, stick_cal.lx_max);
    //     cal_lsy = apply_calibration(raw_lsy, stick_cal.ly_min, stick_cal.ly_center, stick_cal.ly_max);
    //     cal_rsx = apply_calibration(raw_rsx, stick_cal.rx_min, stick_cal.rx_center, stick_cal.rx_max);
    //     cal_rsy = apply_calibration(raw_rsy, stick_cal.ry_min, stick_cal.ry_center, stick_cal.ry_max);
    // } else {
        cal_lsx = apply_deadzone(raw_lsx, 2048);
        cal_lsy = apply_deadzone(raw_lsy, 2048);
        cal_rsx = apply_deadzone(raw_rsx, 2048);
        cal_rsy = apply_deadzone(raw_rsy, 2048);
    // }

    g_stick_data.lsx = cal_lsx & 0xFFF;
    g_stick_data.lsy = cal_lsy & 0xFFF;
    g_stick_data.rsx = cal_rsx & 0xFFF;
    g_stick_data.rsy = cal_rsy & 0xFFF;

    // Log toutes les 500ms (moins de spam)
    if ((xTaskGetTickCount() - last_log_tick) > (500 / portTICK_PERIOD_MS)) {
        last_log_tick = xTaskGetTickCount();
        ESP_LOGI("STICK_POS", "L=(%d,%d)->(%d,%d) | R=(%d,%d)->(%d,%d)",
                 raw_lsx, raw_lsy, g_stick_data.lsx, g_stick_data.lsy,
                 raw_rsx, raw_rsy, g_stick_data.rsx, g_stick_data.rsy);
    }
}

// Tâche de monitoring calibration - remise en place
void calibration_monitor_task(void *pvParameters)
{
    static uint32_t calibrate_pressed_time = 0;

    while (1)
    {
        if (!gpio_get_level(GPIO_BTN_CALIBRATE) && !calibration_in_progress)
        {
            if (calibrate_pressed_time == 0)
            {
                calibrate_pressed_time = xTaskGetTickCount();
                ESP_LOGI("CALIB", "Calibration button pressed - hold for 3 seconds...");
            }
            else if ((xTaskGetTickCount() - calibrate_pressed_time) > (3000 / portTICK_PERIOD_MS))
            {
                ESP_LOGI("CALIB", "3 seconds reached - starting calibration");
                calibrate_sticks();
                calibrate_pressed_time = 0;
            }
        }
        else
        {
            if (calibrate_pressed_time != 0) {
                ESP_LOGI("CALIB", "Calibration button released");
            }
            calibrate_pressed_time = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


// Configuration améliorée des logs
void setup_debug_logging() {
    // Test immédiat
    printf("\n=== ESP32 Pro Controller Starting ===\n");
    
    // Configuration des niveaux de log
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("CALIB", ESP_LOG_INFO);
    esp_log_level_set("STICK_POS", ESP_LOG_INFO);
    esp_log_level_set("app_main", ESP_LOG_INFO);
    esp_log_level_set("rbc_core_ns_start", ESP_LOG_INFO);
    
    ESP_LOGE("DEBUG", "ERROR level test - should always appear");
    ESP_LOGW("DEBUG", "WARNING level test");
    ESP_LOGI("DEBUG", "INFO level test");
    ESP_LOGD("DEBUG", "DEBUG level test");
    
    ESP_LOGI("CALIB", "INFO level test");
    ESP_LOGI("STICK_POS", "INFO level test");
    ESP_LOGI("app_main", "INFO level test");
    ESP_LOGI("rbc_core_ns_start", "INFO level test");
    
    ESP_LOGI("DEBUG", "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI("DEBUG", "Free heap: %d bytes", esp_get_free_heap_size());
    printf("=== Log system ready ===\n\n");
}


void app_main()
{
    // PREMIÈRE CHOSE : configuration des logs
    setup_debug_logging();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const char *TAG = "app_main";
    ESP_LOGI(TAG, "=== Starting ESP32 Switch Pro Controller ===");

    // Configuration GPIO
    ESP_LOGI(TAG, "Configuring GPIO...");
    gpio_config_t io_conf_led = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_led));

    gpio_set_level(GPIO_LED_SYNC, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_L, 1);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 1);
    ESP_LOGI(TAG, "LEDs configured - showing startup state");

    // Configuration ADC
    ESP_LOGI(TAG, "Configuring ADC...");
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_STICK_LX, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_STICK_LY, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_STICK_RX, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_STICK_RY, ADC_ATTEN_DB_11));
    ESP_LOGI(TAG, "ADC configured successfully");

    // Test ADC initial
    uint16_t test_lx = adc1_get_raw(ADC_STICK_LX);
    uint16_t test_ly = adc1_get_raw(ADC_STICK_LY);
    ESP_LOGI(TAG, "Initial ADC test - LX:%d LY:%d", test_lx, test_ly);

    // Configuration GPIO boutons
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "Button GPIO configured");

    // Charger calibration
    ESP_LOGI(TAG, "Loading stick calibration...");
    load_stick_calibration();
    if (stick_cal.calibrated) {
        ESP_LOGI(TAG, "Calibration found and loaded");
        ESP_LOGI(TAG, "Left stick  - X: %d-%d-%d, Y: %d-%d-%d", 
                 stick_cal.lx_min, stick_cal.lx_center, stick_cal.lx_max,
                 stick_cal.ly_min, stick_cal.ly_center, stick_cal.ly_max);
        ESP_LOGI(TAG, "Right stick - X: %d-%d-%d, Y: %d-%d-%d", 
                 stick_cal.rx_min, stick_cal.rx_center, stick_cal.rx_max,
                 stick_cal.ry_min, stick_cal.ry_center, stick_cal.ry_max);
    } else {
        ESP_LOGW(TAG, "No calibration - hold button GPIO15 for 3s to calibrate");
    }

    ESP_LOGI(TAG, "Free heap before RetroBlue: %d bytes", esp_get_free_heap_size());

    // Initialiser RetroBlue
    ESP_LOGI(TAG, "Registering callbacks...");
    rb_register_button_callback(button_task);
    rb_register_stick_callback(stick_task);

    ESP_LOGI(TAG, "Initializing RetroBlue API...");
    if (rb_api_init() != RB_OK) {
        ESP_LOGE(TAG, "RetroBlue API init failed!");
        return;
    }

    if (rb_api_setCore() != RB_OK) {
        ESP_LOGE(TAG, "RetroBlue core set failed!");
        return;
    }

    ESP_LOGI(TAG, "Starting controller...");
    if (rb_api_startController() != RB_OK) {
        ESP_LOGE(TAG, "Controller start failed!");
        return;
    }

    update_switch_battery();
    ESP_LOGI(TAG, "Battery level updated: %d/8", current_battery_level);

    // LEDs finales
    gpio_set_level(GPIO_LED_SYNC, 1);
    gpio_set_level(GPIO_LED_CALIBRATE_L, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 0);

    ESP_LOGI(TAG, "=== SWITCH PRO CONTROLLER READY ===");
    ESP_LOGI(TAG, "Battery: %d/8, Free heap: %d bytes", current_battery_level, esp_get_free_heap_size());
    ESP_LOGI(TAG, "Hold calibration button (GPIO15) for 3 seconds to start calibration");

    // Création de la tâche de monitoring de calibration
    ESP_LOGI(TAG, "Creating calibration monitor task...");
    BaseType_t task_result = xTaskCreatePinnedToCore(
        calibration_monitor_task, 
        "Calibration Monitor", 
        2048, 
        NULL, 
        1, 
        NULL, 
        1
    );
    
    if (task_result == pdPASS) {
        ESP_LOGI(TAG, "Calibration monitor task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create calibration monitor task");
    }

    ESP_LOGI(TAG, "System initialization complete");
}