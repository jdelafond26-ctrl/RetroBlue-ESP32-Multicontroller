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

#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_LED_0) | (1ULL << GPIO_LED_1) | (1ULL << GPIO_LED_2)| (1ULL << GPIO_LED_3))
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_BTN_DU) | (1ULL << GPIO_BTN_DD) | (1ULL << GPIO_BTN_DL) | (1ULL << GPIO_BTN_DR) | (1ULL << GPIO_BTN_STICKL) | (1ULL << GPIO_BTN_SELECT) | (1ULL << GPIO_BTN_L) | (1ULL << GPIO_BTN_ZL) | (1ULL << GPIO_BTN_STICKR) | (1ULL << GPIO_BTN_R) | (1ULL << GPIO_BTN_ZR) | (1ULL << GPIO_BTN_A) | (1ULL << GPIO_BTN_B) | (1ULL << GPIO_BTN_X) | (1ULL << GPIO_BTN_Y) | (1ULL << GPIO_BTN_START) | (1ULL << GPIO_BTN_HOME) | (1ULL << GPIO_BTN_CAPTURE))

#define CAL_MARGIN 50 // marge pour éviter min/max trop serrés

// Variables
uint32_t regread = 0;
const uint16_t DEADZONE = 80;
uint8_t current_battery_level = 0x8;

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

// SUPPRESSION DE LA REDONDANCE : On garde seulement dans button_task
void button_task()
{
    regread = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_SEL;

    // Boutons Switch Pro Controller
    g_button_data.d_up = getbit(regread, GPIO_BTN_DU);
    g_button_data.d_down = getbit(regread, GPIO_BTN_DD);
    g_button_data.d_left = getbit(regread, GPIO_BTN_DL);
    g_button_data.d_right = getbit(regread, GPIO_BTN_DR);

    g_button_data.b_right = getbit(regread, GPIO_BTN_A);
    g_button_data.b_down = getbit(regread, GPIO_BTN_B);
    g_button_data.b_up = getbit(regread, GPIO_BTN_X);
    g_button_data.b_left = getbit(regread, GPIO_BTN_Y);

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
    uint16_t raw_lsx = adc1_get_raw(ADC_STICK_LX);
    uint16_t raw_lsy = adc1_get_raw(ADC_STICK_LY);
    uint16_t raw_rsx = adc1_get_raw(ADC_STICK_RX);
    uint16_t raw_rsy = adc1_get_raw(ADC_STICK_RY);

    uint16_t cal_lsx, cal_lsy, cal_rsx, cal_rsy;

    cal_lsx = apply_deadzone(raw_lsx, 2048);
    cal_lsy = apply_deadzone(raw_lsy, 2048);
    cal_rsx = apply_deadzone(raw_rsx, 2048);
    cal_rsy = apply_deadzone(raw_rsy, 2048);

    g_stick_data.lsx = cal_lsx & 0xFFF;
    g_stick_data.lsy = cal_lsy & 0xFFF;
    g_stick_data.rsx = cal_rsx & 0xFFF;
    g_stick_data.rsy = cal_rsy & 0xFFF;
}

void app_main()
{
    // PREMIÈRE CHOSE : configuration des logs
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

    gpio_set_level(GPIO_LED_0, 1);
    gpio_set_level(GPIO_LED_1, 0);
    gpio_set_level(GPIO_LED_2, 0);
    gpio_set_level(GPIO_LED_3, 1);
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
    gpio_set_level(GPIO_LED_0, 1);
    gpio_set_level(GPIO_LED_1, 1);
    gpio_set_level(GPIO_LED_2, 1);
    gpio_set_level(GPIO_LED_3, 1);

    ESP_LOGI(TAG, "=== SWITCH PRO CONTROLLER READY ===");
    ESP_LOGI(TAG, "Battery: %d/8, Free heap: %d bytes", current_battery_level, esp_get_free_heap_size());
}