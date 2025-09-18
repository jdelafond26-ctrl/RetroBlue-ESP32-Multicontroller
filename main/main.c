#include "main.h"

// ADC channels
#define ADC_STICK_LX ADC1_CHANNEL_0     // VP
#define ADC_STICK_LY ADC1_CHANNEL_3     // VN
#define ADC_STICK_RX ADC1_CHANNEL_6     // D34
#define ADC_STICK_RY ADC1_CHANNEL_7     // D35

// GPIO Boutons
#define GPIO_BTN_DU GPIO_NUM_25         // D25
#define GPIO_BTN_DD GPIO_NUM_26         // D26
#define GPIO_BTN_DL GPIO_NUM_27         // D27
#define GPIO_BTN_DR GPIO_NUM_14         // D14
#define GPIO_BTN_SELECT GPIO_NUM_12     // D12 (-)
#define GPIO_BTN_START GPIO_NUM_13      // D13 (+)
#define GPIO_BTN_L GPIO_NUM_23          // D23
#define GPIO_BTN_R GPIO_NUM_22          // D22
#define GPIO_BTN_ZL GPIO_NUM_32         // D32
#define GPIO_BTN_ZR GPIO_NUM_33         // D33
#define GPIO_BTN_STICKL GPIO_NUM_3      // RX0
#define GPIO_BTN_STICKR GPIO_NUM_21     // D21
#define GPIO_BTN_B GPIO_NUM_19          // D19
#define GPIO_BTN_A GPIO_NUM_18          // D18
#define GPIO_BTN_Y GPIO_NUM_5           // D5
#define GPIO_BTN_X GPIO_NUM_17          // TX2
#define GPIO_BTN_HOME GPIO_NUM_16       // RX2
#define GPIO_BTN_CAPTURE GPIO_NUM_4     // D4

// Calibration
#define GPIO_BTN_CALIBRATE GPIO_NUM_15  // D15
#define GPIO_LED_SYNC GPIO_NUM_1        // D1
#define GPIO_LED_CALIBRATE_L GPIO_NUM_0 // D0
#define GPIO_LED_CALIBRATE_R GPIO_NUM_2 // D2

#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_LED_SYNC) | (1ULL << GPIO_LED_CALIBRATE_L) | (1ULL << GPIO_LED_CALIBRATE_R))
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_BTN_CALIBRATE) | (1ULL << GPIO_BTN_DU) | (1ULL << GPIO_BTN_DD) | (1ULL << GPIO_BTN_DL) | (1ULL << GPIO_BTN_DR) | (1ULL << GPIO_BTN_STICKL) | (1ULL << GPIO_BTN_SELECT) | (1ULL << GPIO_BTN_L) | (1ULL << GPIO_BTN_ZL) | (1ULL << GPIO_BTN_STICKR) | (1ULL << GPIO_BTN_R) | (1ULL << GPIO_BTN_ZR) | (1ULL << GPIO_BTN_A) | (1ULL << GPIO_BTN_B) | (1ULL << GPIO_BTN_X) | (1ULL << GPIO_BTN_Y) | (1ULL << GPIO_BTN_START) | (1ULL << GPIO_BTN_HOME) | (1ULL << GPIO_BTN_CAPTURE))

// Variables
uint32_t regread = 0;
const uint16_t DEADZONE = 80;
uint8_t current_battery_level = 0x8;


// Calibration des sticks
typedef struct {
    uint16_t lx_min, lx_center, lx_max;
    uint16_t ly_min, ly_center, ly_max;
    uint16_t rx_min, rx_center, rx_max;
    uint16_t ry_min, ry_center, ry_max;
    bool calibrated;
} stick_calibration_t;

stick_calibration_t stick_cal = {0};
bool calibration_in_progress = false;

// Fonctions utilitaires
bool getbit(uint32_t bytes, uint8_t bit) {
    return !((bytes >> bit) & 0x1);
}

uint16_t apply_deadzone(uint16_t raw, uint16_t center) {
    int32_t diff = (int32_t)raw - (int32_t)center;
    if (abs(diff) < DEADZONE) return center;
    return raw;
}

// Fonction pour mettre à jour le niveau de batterie dans les données Switch
void update_switch_battery() {
    // Mettre à jour la structure Switch avec le vrai niveau de batterie
    // Cette fonction doit être appelée périodiquement
    extern ns_controller_data_s ns_controller_data; // Référence à la structure Switch
    ns_controller_data.battery_level_full = current_battery_level;
}

// Fonctions calibration
void save_stick_calibration() {
    const char* TAG = "save_stick_calibration";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("calibration", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS for calibration");
        return;
    }
    nvs_set_blob(my_handle, "stick_cal", &stick_cal, sizeof(stick_calibration_t));
    nvs_commit(my_handle);
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Stick calibration saved");
}

void load_stick_calibration() {
    const char* TAG = "load_stick_calibration";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("calibration", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No calibration found, using defaults");
        stick_cal.lx_center = stick_cal.ly_center = stick_cal.rx_center = stick_cal.ry_center = 2048;
        stick_cal.lx_min = stick_cal.ly_min = stick_cal.rx_min = stick_cal.ry_min = 200;
        stick_cal.lx_max = stick_cal.ly_max = stick_cal.rx_max = stick_cal.ry_max = 3900;
        stick_cal.calibrated = false;
        return;
    }

    size_t required_size = sizeof(stick_calibration_t);
    err = nvs_get_blob(my_handle, "stick_cal", &stick_cal, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No calibration found, using defaults");
        stick_cal.lx_center = stick_cal.ly_center = stick_cal.rx_center = stick_cal.ry_center = 2048;
        stick_cal.lx_min = stick_cal.ly_min = stick_cal.rx_min = stick_cal.ry_min = 200;
        stick_cal.lx_max = stick_cal.ly_max = stick_cal.rx_max = stick_cal.ry_max = 3900;
        stick_cal.calibrated = false;
    } else {
        ESP_LOGI(TAG, "Stick calibration loaded successfully");
    }
    nvs_close(my_handle);
}

void calibrate_sticks() {
    const char* TAG = "calibrate_sticks";
    ESP_LOGI(TAG, "Starting stick calibration...");
    
    calibration_in_progress = true;
    
    // Étape 1: Centre des sticks
    ESP_LOGI(TAG, "Step 1: Center both sticks and press calibrate button");
    gpio_set_level(GPIO_LED_CALIBRATE_L, 1);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 1);
    
    while (gpio_get_level(GPIO_BTN_CALIBRATE) == 1) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    while (gpio_get_level(GPIO_BTN_CALIBRATE) == 0) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    stick_cal.lx_center = adc1_get_raw(ADC_STICK_LX);
    stick_cal.ly_center = adc1_get_raw(ADC_STICK_LY);
    stick_cal.rx_center = adc1_get_raw(ADC_STICK_RX);
    stick_cal.ry_center = adc1_get_raw(ADC_STICK_RY);
    
    ESP_LOGI(TAG, "Center recorded: LX=%d, LY=%d, RX=%d, RY=%d", 
             stick_cal.lx_center, stick_cal.ly_center, stick_cal.rx_center, stick_cal.ry_center);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Étape 2: Stick gauche
    ESP_LOGI(TAG, "Step 2: Move LEFT stick in full circles for 8 seconds");
    gpio_set_level(GPIO_LED_CALIBRATE_R, 1);
    
    stick_cal.lx_min = stick_cal.lx_max = stick_cal.lx_center;
    stick_cal.ly_min = stick_cal.ly_max = stick_cal.ly_center;
    
    uint32_t start_time = xTaskGetTickCount();
    bool led_state = false;
    
    while ((xTaskGetTickCount() - start_time) < (8000 / portTICK_PERIOD_MS)) {
        if ((xTaskGetTickCount() - start_time) % (200 / portTICK_PERIOD_MS) == 0) {
            led_state = !led_state;
            gpio_set_level(GPIO_LED_CALIBRATE_L, led_state);
        }
        
        uint16_t lx = adc1_get_raw(ADC_STICK_LX);
        uint16_t ly = adc1_get_raw(ADC_STICK_LY);
        
        if (lx < stick_cal.lx_min) stick_cal.lx_min = lx;
        if (lx > stick_cal.lx_max) stick_cal.lx_max = lx;
        if (ly < stick_cal.ly_min) stick_cal.ly_min = ly;
        if (ly > stick_cal.ly_max) stick_cal.ly_max = ly;
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "Left stick: X=[%d-%d-%d], Y=[%d-%d-%d]", 
             stick_cal.lx_min, stick_cal.lx_center, stick_cal.lx_max,
             stick_cal.ly_min, stick_cal.ly_center, stick_cal.ly_max);
    
    // Étape 3: Stick droit
    ESP_LOGI(TAG, "Step 3: Move RIGHT stick in full circles for 8 seconds");
    gpio_set_level(GPIO_LED_CALIBRATE_L, 1);
    
    stick_cal.rx_min = stick_cal.rx_max = stick_cal.rx_center;
    stick_cal.ry_min = stick_cal.ry_max = stick_cal.ry_center;
    
    start_time = xTaskGetTickCount();
    led_state = false;
    
    while ((xTaskGetTickCount() - start_time) < (8000 / portTICK_PERIOD_MS)) {
        if ((xTaskGetTickCount() - start_time) % (200 / portTICK_PERIOD_MS) == 0) {
            led_state = !led_state;
            gpio_set_level(GPIO_LED_CALIBRATE_R, led_state);
        }
        
        uint16_t rx = adc1_get_raw(ADC_STICK_RX);
        uint16_t ry = adc1_get_raw(ADC_STICK_RY);
        
        if (rx < stick_cal.rx_min) stick_cal.rx_min = rx;
        if (rx > stick_cal.rx_max) stick_cal.rx_max = rx;
        if (ry < stick_cal.ry_min) stick_cal.ry_min = ry;
        if (ry > stick_cal.ry_max) stick_cal.ry_max = ry;
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "Right stick: X=[%d-%d-%d], Y=[%d-%d-%d]", 
             stick_cal.rx_min, stick_cal.rx_center, stick_cal.rx_max,
             stick_cal.ry_min, stick_cal.ry_center, stick_cal.ry_max);
    
    // Finaliser
    stick_cal.calibrated = true;
    save_stick_calibration();
    
    // Clignotement de confirmation
    for (int i = 0; i < 10; i++) {
        gpio_set_level(GPIO_LED_CALIBRATE_L, i % 2);
        gpio_set_level(GPIO_LED_CALIBRATE_R, i % 2);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    gpio_set_level(GPIO_LED_CALIBRATE_L, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 0);
    calibration_in_progress = false;
    
    ESP_LOGI(TAG, "Calibration completed and saved!");
}

uint16_t apply_calibration(uint16_t raw, uint16_t min_val, uint16_t center, uint16_t max_val) {
    if (!stick_cal.calibrated) {
        return apply_deadzone(raw, 2048);
    }
    
    int32_t diff = (int32_t)raw - (int32_t)center;
    if (abs(diff) < DEADZONE) {
        return 2048;
    }
    
    if (raw < center) {
        if (raw <= min_val) return 0;
        return (uint16_t)((raw - min_val) * 2048 / (center - min_val));
    } else {
        if (raw >= max_val) return 4095;
        return 2048 + (uint16_t)((raw - center) * 2047 / (max_val - center));
    }
}

// Tâches principales
void button_task() {
    regread = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_SEL;

    // Gestion calibration (3 secondes)
    static uint32_t calibrate_pressed_time = 0;
    if (!gpio_get_level(GPIO_BTN_CALIBRATE) && !calibration_in_progress) {
        if (calibrate_pressed_time == 0) {
            calibrate_pressed_time = xTaskGetTickCount();
        } else if ((xTaskGetTickCount() - calibrate_pressed_time) > (3000 / portTICK_PERIOD_MS)) {
            calibrate_sticks();
            calibrate_pressed_time = 0;
        }
    } else {
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

void stick_task() {
    if (calibration_in_progress) return;
    
    uint16_t raw_lsx = adc1_get_raw(ADC_STICK_LX);
    uint16_t raw_lsy = adc1_get_raw(ADC_STICK_LY);
    uint16_t raw_rsx = adc1_get_raw(ADC_STICK_RX);
    uint16_t raw_rsy = adc1_get_raw(ADC_STICK_RY);

    if (stick_cal.calibrated) {
        raw_lsx = apply_calibration(raw_lsx, stick_cal.lx_min, stick_cal.lx_center, stick_cal.lx_max);
        raw_lsy = apply_calibration(raw_lsy, stick_cal.ly_min, stick_cal.ly_center, stick_cal.ly_max);
        raw_rsx = apply_calibration(raw_rsx, stick_cal.rx_min, stick_cal.rx_center, stick_cal.rx_max);
        raw_rsy = apply_calibration(raw_rsy, stick_cal.ry_min, stick_cal.ry_center, stick_cal.ry_max);
    } else {
        raw_lsx = apply_deadzone(raw_lsx, 2048);
        raw_lsy = apply_deadzone(raw_lsy, 2048);
        raw_rsx = apply_deadzone(raw_rsx, 2048);
        raw_rsy = apply_deadzone(raw_rsy, 2048);
    }

    g_stick_data.lsx = raw_lsx & 0xFFF;
    g_stick_data.lsy = raw_lsy & 0xFFF;
    g_stick_data.rsx = raw_rsx & 0xFFF;
    g_stick_data.rsy = raw_rsy & 0xFFF;
}

void app_main() {
    const char *TAG = "app_main";

    // Configuration GPIO
    gpio_config_t io_conf_led = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf_led);

    gpio_set_level(GPIO_LED_SYNC, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_L, 1);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 1);

    // Configuration ADC
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_STICK_LX, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_STICK_LY, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_STICK_RX, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_STICK_RY, ADC_ATTEN_DB_11));

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    // Charger calibration
    load_stick_calibration();
    if (stick_cal.calibrated) {
        ESP_LOGI(TAG, "Stick calibration loaded");
    } else {
        ESP_LOGW(TAG, "No calibration - hold button 3s to calibrate");
    }

    // Initialiser RetroBlue
    rb_register_button_callback(button_task);
    rb_register_stick_callback(stick_task);
    
    rb_api_init();
    rb_api_setCore();
    rb_api_startController();

    update_switch_battery(); // Synchroniser le niveau de batterie avec les données Switch
    
    gpio_set_level(GPIO_LED_SYNC, 1);
    gpio_set_level(GPIO_LED_CALIBRATE_L, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 0);
    
    ESP_LOGI(TAG, "Switch Pro Controller ready - Battery: %d/8", current_battery_level);
    ESP_LOGI(TAG, "Hold calibration button (GPIO15) 3s to start calibration");
}
