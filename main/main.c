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
    // Mettre à jour la structure Switch avec le vrai niveau de batterie
    // Cette fonction doit être appelée périodiquement
    extern ns_controller_data_s ns_controller_data; // Référence à la structure Switch
    ns_controller_data.battery_level_full = current_battery_level;
}

// Fonctions calibration
void save_stick_calibration()
{
    const char *TAG = "save_stick_calibration";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("calibration", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS for calibration");
        return;
    }
    nvs_set_blob(my_handle, "stick_cal", &stick_cal, sizeof(stick_calibration_t));
    nvs_commit(my_handle);
    nvs_close(my_handle);
    ESP_LOGI(TAG, "Stick calibration saved");
}

void load_stick_calibration()
{
    const char *TAG = "load_stick_calibration";
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("calibration", NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No calibration found, using defaults");
        stick_cal.lx_center = stick_cal.ly_center = stick_cal.rx_center = stick_cal.ry_center = 2048;
        stick_cal.lx_min = stick_cal.ly_min = stick_cal.rx_min = stick_cal.ry_min = 200;
        stick_cal.lx_max = stick_cal.ly_max = stick_cal.rx_max = stick_cal.ry_max = 3900;
        stick_cal.calibrated = false;
        return;
    }

    size_t required_size = sizeof(stick_calibration_t);
    err = nvs_get_blob(my_handle, "stick_cal", &stick_cal, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No calibration found, using defaults");
        stick_cal.lx_center = stick_cal.ly_center = stick_cal.rx_center = stick_cal.ry_center = 2048;
        stick_cal.lx_min = stick_cal.ly_min = stick_cal.rx_min = stick_cal.ry_min = 200;
        stick_cal.lx_max = stick_cal.ly_max = stick_cal.rx_max = stick_cal.ry_max = 3900;
        stick_cal.calibrated = false;
    }
    else
    {
        ESP_LOGI(TAG, "Stick calibration loaded successfully");
    }
    nvs_close(my_handle);
}

// Fonction pour clignoter une LED pendant un temps donné (ms) avec intervalle de clignotement
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
    gpio_set_level(gpio, 0); // éteindre LED à la fin
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
    gpio_set_level(GPIO_LED_CALIBRATE_R, 0); // éteindre LED à la fin
}

void measure_stick_center(uint8_t adc_x, uint8_t adc_y, uint16_t *center_x, uint16_t *center_y, uint8_t led_gpio) {
    const uint16_t STABILITY_THRESHOLD = 20; // tolérance ADC réaliste
    const int NUM_SAMPLES = 100;             // nombre de mesures valides à capturer
    const uint32_t TIMEOUT_MS = 5000;        // timeout global en ms

    ESP_LOGI("CALIB", "Get ready! Stick center will be measured in 3 seconds...");
    gpio_set_level(led_gpio, 1); // LED fixe pour signal repos
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_LOGI("CALIB", "Measuring stick center now, hold stick steady!");
    uint32_t start_time = xTaskGetTickCount();

    uint32_t sum_x = 0, sum_y = 0;
    uint16_t last_x = adc1_get_raw(adc_x);
    uint16_t last_y = adc1_get_raw(adc_y);
    bool led_state = false;
    int valid_samples = 0;

    while (valid_samples < NUM_SAMPLES) {
        // Timeout global pour éviter blocage
        if ((xTaskGetTickCount() - start_time) > (TIMEOUT_MS / portTICK_PERIOD_MS)) {
            ESP_LOGW("CALIB", "Timeout reached during center measurement, using current average.");
            break;
        }

        // Lecture ADC
        uint16_t val_x = adc1_get_raw(adc_x);
        uint16_t val_y = adc1_get_raw(adc_y);

        // Vérifier stabilité
        if (abs((int)val_x - (int)last_x) > STABILITY_THRESHOLD ||
            abs((int)val_y - (int)last_y) > STABILITY_THRESHOLD) {
            last_x = val_x;
            last_y = val_y;
            vTaskDelay(20 / portTICK_PERIOD_MS);
            continue; // Ignorer cette mesure
        }

        // Ajouter à la somme si stable
        sum_x += val_x;
        sum_y += val_y;
        last_x = val_x;
        last_y = val_y;
        valid_samples++;

        // Clignotement LED rapide pendant la mesure
        led_state = !led_state;
        gpio_set_level(led_gpio, led_state);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    // Calculer le centre
    if (valid_samples > 0) {
        *center_x = sum_x / valid_samples;
        *center_y = sum_y / valid_samples;
    } else {
        // Fallback si aucune mesure stable
        *center_x = last_x;
        *center_y = last_y;
    }

    gpio_set_level(led_gpio, 0); // éteindre LED
    ESP_LOGI("CALIB", "Center captured: X=%d, Y=%d", *center_x, *center_y);
}

// Fonction pour mesurer min/max d’un stick pendant qu’on fait des cercles
void measure_stick_circles(uint8_t adc_x, uint8_t adc_y, uint16_t *min_x, uint16_t *max_x, uint16_t *min_y, uint16_t *max_y, uint8_t led_gpio)
{
    *min_x = *max_x = adc1_get_raw(adc_x);
    *min_y = *max_y = adc1_get_raw(adc_y);
    uint32_t start_time = xTaskGetTickCount();
    bool led_state = false;

    while ((xTaskGetTickCount() - start_time) < (8000 / portTICK_PERIOD_MS))
    {
        if ((xTaskGetTickCount() - start_time) % (200 / portTICK_PERIOD_MS) == 0)
        {
            led_state = !led_state;
            gpio_set_level(led_gpio, led_state);
        }
        uint16_t val_x = adc1_get_raw(adc_x);
        uint16_t val_y = adc1_get_raw(adc_y);

        if (val_x < *min_x)
            *min_x = val_x;
        if (val_x > *max_x)
            *max_x = val_x;
        if (val_y < *min_y)
            *min_y = val_y;
        if (val_y > *max_y)
            *max_y = val_y;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Fonction principale de calibration
void calibrate_sticks()
{
    const char *TAG = "calibrate_sticks";
    calibration_in_progress = true;

    // Début : L+R fixes 3 s
    ESP_LOGI(TAG, "Calibration starts in 3 seconds...");
    gpio_set_level(GPIO_LED_CALIBRATE_L, 1);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    gpio_set_level(GPIO_LED_CALIBRATE_L, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 0);

    // --- Stick gauche ---
    ESP_LOGI(TAG, "Move LEFT stick in circles for 8 seconds");
    measure_stick_circles(ADC_STICK_LX, ADC_STICK_LY, &stick_cal.lx_min, &stick_cal.lx_max, &stick_cal.ly_min, &stick_cal.ly_max, GPIO_LED_CALIBRATE_L);

    ESP_LOGI(TAG, "Release LEFT stick for 3 seconds before center measurement");
    gpio_set_level(GPIO_LED_CALIBRATE_L, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Measuring LEFT stick center");
    measure_stick_center(ADC_STICK_LX, ADC_STICK_LY, &stick_cal.lx_center, &stick_cal.ly_center, GPIO_LED_CALIBRATE_L);

    // Signal changement stick (LED L+R clignotent rapide)
    ESP_LOGI(TAG, "Switching to RIGHT stick...");
    blink_leds(3000, 50);

    // --- Stick droit ---
    ESP_LOGI(TAG, "Move RIGHT stick in circles for 8 seconds");
    measure_stick_circles(ADC_STICK_RX, ADC_STICK_RY, &stick_cal.rx_min, &stick_cal.rx_max, &stick_cal.ry_min, &stick_cal.ry_max, GPIO_LED_CALIBRATE_R);

    ESP_LOGI(TAG, "Release RIGHT stick for 3 seconds before center measurement");
    gpio_set_level(GPIO_LED_CALIBRATE_R, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Measuring RIGHT stick center");
    measure_stick_center(ADC_STICK_RX, ADC_STICK_RY, &stick_cal.rx_center, &stick_cal.ry_center, GPIO_LED_CALIBRATE_R);

    // Clignotement final pour confirmer
    ESP_LOGI(TAG, "Calibration complete, blinking both LEDs...");
    for (int i = 0; i < 10; i++)
    {
        gpio_set_level(GPIO_LED_CALIBRATE_L, i % 2);
        gpio_set_level(GPIO_LED_CALIBRATE_R, i % 2);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    gpio_set_level(GPIO_LED_CALIBRATE_L, 0);
    gpio_set_level(GPIO_LED_CALIBRATE_R, 0);

    stick_cal.calibrated = true;
    save_stick_calibration();
    calibration_in_progress = false;
    ESP_LOGI(TAG, "Calibration saved successfully!");
}

uint16_t apply_calibration(uint16_t raw, uint16_t min_val, uint16_t center, uint16_t max_val)
{
    if (!stick_cal.calibrated)
    {
        return apply_deadzone(raw, 2048);
    }

    int32_t diff = (int32_t)raw - (int32_t)center;
    if (abs(diff) < DEADZONE)
    {
        return 2048; // valeur neutre
    }

    if (raw < center)
    {
        if (raw <= min_val)
            return 0;
        // map proportionnellement vers la gauche
        return 2048 - (uint16_t)((center - raw) * 2048 / (center - min_val));
    }
    else
    {
        if (raw >= max_val)
            return 4095;
        // map proportionnellement vers la droite
        return 2048 + (uint16_t)((raw - center) * 2047 / (max_val - center));
    }
}

// Tâches principales
void button_task()
{
    regread = REG_READ(GPIO_IN_REG) & GPIO_INPUT_PIN_SEL;

    // Gestion calibration (3 secondes)
    static uint32_t calibrate_pressed_time = 0;
    if (!gpio_get_level(GPIO_BTN_CALIBRATE) && !calibration_in_progress)
    {
        if (calibrate_pressed_time == 0)
        {
            calibrate_pressed_time = xTaskGetTickCount();
        }
        else if ((xTaskGetTickCount() - calibrate_pressed_time) > (3000 / portTICK_PERIOD_MS))
        {
            calibrate_sticks();
            calibrate_pressed_time = 0;
        }
    }
    else
    {
        calibrate_pressed_time = 0;
    }

    if (calibration_in_progress)
        return;

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
    if (calibration_in_progress)
        return;

    uint16_t raw_lsx = adc1_get_raw(ADC_STICK_LX);
    uint16_t raw_lsy = adc1_get_raw(ADC_STICK_LY);
    uint16_t raw_rsx = adc1_get_raw(ADC_STICK_RX);
    uint16_t raw_rsy = adc1_get_raw(ADC_STICK_RY);

    if (stick_cal.calibrated)
    {
        raw_lsx = apply_calibration(raw_lsx, stick_cal.lx_min, stick_cal.lx_center, stick_cal.lx_max);
        raw_lsy = apply_calibration(raw_lsy, stick_cal.ly_min, stick_cal.ly_center, stick_cal.ly_max);
        raw_rsx = apply_calibration(raw_rsx, stick_cal.rx_min, stick_cal.rx_center, stick_cal.rx_max);
        raw_rsy = apply_calibration(raw_rsy, stick_cal.ry_min, stick_cal.ry_center, stick_cal.ry_max);
    }
    else
    {
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
                ESP_LOGI("CALIB", "Button pressed, starting timer");
            }
            else if ((xTaskGetTickCount() - calibrate_pressed_time) > (3000 / portTICK_PERIOD_MS))
            {
                ESP_LOGI("CALIB", "3 seconds reached, starting calibration");
                calibrate_sticks();
                calibrate_pressed_time = 0;
            }
        }
        else
        {
            calibrate_pressed_time = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    const char *TAG = "app_main";

    // Configuration GPIO
    gpio_config_t io_conf_led = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
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
        .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io_conf);

    // Charger calibration
    load_stick_calibration();
    if (stick_cal.calibrated)
    {
        ESP_LOGI(TAG, "Stick calibration loaded");
    }
    else
    {
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

    xTaskCreatePinnedToCore(calibration_monitor_task, "Calibration Monitor", 2048, NULL, 1, NULL, 1);
}
