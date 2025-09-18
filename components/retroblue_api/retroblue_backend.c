#include "retroblue_backend.h"

GamepadButtonData g_button_data = {};
GamepadStickData g_stick_data = {};
rb_param_s rb_params = {};

void rb_button_reset() {
    memset(&g_button_data, 0, sizeof(GamepadButtonData));
}