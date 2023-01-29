/*
Copyright 2022 Thomas Mørch (tbowmo)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include QMK_KEYBOARD_H

#ifdef BLUETOOTH_ENABLE
#include "iton_bt.h"
#include "outputselect.h"
#endif

enum tbowmo_layer_names {
  _BASE,
  _FN1,
  _FN2,
};

#define KC_FN1 LT(_FN1, KC_APP)
#define KC_FN2 LT(_FN2, KC_SCRL)
#define KC_FN1_ALT LT(_FN1, KC_CAPS)
#define KC_TASK LGUI(KC_TAB)        // Task viewer
#define KC_FLXP LGUI(KC_E)          // Windows file explorer

enum bt_keys {
  KC_BT1 = SAFE_RANGE,
  KC_BT2,
  KC_BT3,
  KC_USB,
  KC_PAIR,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/*			+--------------------------------------------------------------------------+----------------+
				| ESC |  | F1 | F2 | F3 | F4 | | F5 | F6 | F7 | F8 | | F9| F10| F11| F12|  | |PSCR|????|PAUS|
				+--------------------------------------------------------------------------+------|----|----|
 				|  ~  |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 |  0 |  - |  = | BACKSP | |INS |SCRL|PGUP|
 				+--------------------------------------------------------------------------+------|----|----|
 				|  TAB  |  Q |  W |  E |  R |  T |  Y |  U |  I |  O |  P |  [ |  ] |      | |DEL |END |PGDN|
 				+--------------------------------------------------------------------  RET |------|----|----|
 				| CAPSLCK  |  A |  S |  D |  F |  G |  H |  J |  K |  L | ; | ' |  # |     |                |
 				+--------------------------------------------------------------------------+      |----|    |
 				| LSHIFT | \ |  Z |  X |  C |  V |  B |  N |  M | , | . |  / |   RSHIFT    |      | UP |    |
 				+--------------------------------------------------------------------------+------|----|----|
 				|LCTRL| LGUI| LALT |            SPACE            | RALT| RGUI | FN | RCTRL | |LFT |DWN |RGT |
 				+--------------------------------------------------------------------------+----------------+
*/
    [_BASE] = LAYOUT_tkl_iso(
        KC_ESC,           KC_F1,   KC_F2,   KC_F3,    KC_F4,    KC_F5,    KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,   KC_F11,   KC_F12,        KC_PSCR,   KC_FN2,   KC_MUTE,
        KC_GRV,  KC_1,    KC_2,    KC_3,    KC_4,     KC_5,     KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_MINS,  KC_EQL,   KC_BSPC,       KC_INS,    KC_HOME,  KC_PGUP,
        KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,     KC_T,     KC_Y,     KC_U,     KC_I,     KC_O,     KC_P,     KC_LBRC,  KC_RBRC,                 KC_DEL,    KC_END,   KC_PGDN,
     KC_FN1_ALT, KC_A,    KC_S,    KC_D,    KC_F,     KC_G,     KC_H,     KC_J,     KC_K,     KC_L,     KC_SCLN,  KC_QUOT,  KC_NUHS,  KC_ENT,
        KC_LSFT, KC_NUBS, KC_Z,    KC_X,    KC_C,     KC_V,     KC_B,     KC_N,     KC_M,     KC_COMM,  KC_DOT,   KC_SLSH,  KC_RSFT,                            KC_UP,
        KC_LCTL, KC_LGUI, KC_LALT,                    KC_SPC,                                           KC_RALT,  KC_RGUI,  KC_FN1,   KC_RCTL,       KC_LEFT,   KC_DOWN,  KC_RGHT
    ),

    [_FN1] = LAYOUT_tkl_iso(
        QK_BOOT,          KC_BRID, KC_BRIU, KC_TASK,  KC_FLXP,  RGB_VAD,  RGB_VAI,  KC_MPRV,  KC_MPLY,  KC_MNXT,  KC_MUTE,  KC_VOLD,  KC_VOLU,       _______,   _______,  RGB_TOG,
        _______,  KC_BT1,  KC_BT2,  KC_BT3,  KC_USB,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,       RGB_VAI,   RGB_SAI,  RGB_HUI,
        _______, _______, _______, _______, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,                 RGB_VAD,   RGB_SAD,  RGB_HUD,
        _______, _______, _______, _______, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,
        _______, _______, _______, _______, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  KC_WREF,                            KC_VOLU,
        _______, _______, _______,                    _______,                                          _______,  _______,  _______,  KC_MPLY,       KC_MPRV,   KC_VOLD,  KC_MNXT
    ),
    [_FN2] = LAYOUT_tkl_iso(
        _______,          KC_F13,   KC_F14,   KC_F15,   KC_F16,   KC_F17,   KC_F18,   KC_F19,   KC_F20,   KC_F21,   KC_F22,   KC_F23,   KC_F24,      _______,  _______,  KC_SLEP,
        _______, _______, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,     _______,  _______,  _______,
        _______, _______, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,               _______,  _______,  _______,
        _______, _______, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,
        _______, _______, _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,  _______,                         _______,
        _______, _______, _______,                      _______,                                          _______,  _______,  _______,  _______,     RGB_RMOD, _______,  RGB_MOD
    ),
};


typedef union {
  uint32_t raw;
  struct {
    uint8_t bt_profile;
  };
} kb_config_t;

kb_config_t kb_config;

enum profile_states_t {
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_PAIRING,
    STATE_DISCONNECTED,
};

enum profiles_t {
    PROFILE_BT_1 = 0,
    PROFILE_BT_2 = 1,
    PROFILE_BT_3 = 2,
    PROFILE_USB = 3,
};

#define PAIRING_LONG_PRESS  3000
#define DELAY_SLOW_BLINKING 600
#define DELAY_FAST_BLINKING 300

static bool bluetooth_enabled = false;
static bool profile_led = false;
static uint8_t current_profile_state = STATE_DISCONNECTED;
static deferred_token deferred_bt_pairing = INVALID_DEFERRED_TOKEN;
static uint16_t key_pressed_time;

/**
 * Deferred execution for entering pairing mode
*/
uint32_t start_pairing(uint32_t trigger_time, void *cb_arg) {
    /* Initiate pairing mode */
    iton_bt_enter_pairing();
    return 0;
}

/**
 * Deferred execution for LED blinking
*/
uint32_t led_blinking(uint32_t trigger_time, void *cb_arg) {
    profile_led = !profile_led;

    switch (current_profile_state) {
        case STATE_PAIRING:
            return DELAY_SLOW_BLINKING;
        case STATE_DISCONNECTED:
            if (key_pressed_time == 0 || timer_elapsed32(key_pressed_time) > 3000) {
                key_pressed_time = 0;
                profile_led = false;
            }
            return DELAY_FAST_BLINKING;
        case STATE_CONNECTING:
            current_profile_state = STATE_CONNECTED;
            // static led on during bootup in bt mode for 1 second
            profile_led = true;
            return 1000;
        case STATE_CONNECTED:
        default:
            profile_led = false;
            return 1000;
    }
}

/**
 * Internal profile selection, storing the selected profile
 * in eeprom, so it can be retrieved on bootup for flashing correct
 * indicators
*/
void select_profile(uint8_t profile) {
    dprintf("profile %u, enabled %u", profile, bluetooth_enabled);
    if (bluetooth_enabled) {
        if (profile < PROFILE_USB) {
            set_output(OUTPUT_BLUETOOTH);
            // Start pairing in 2 seconds (Will be cancelled on key release)
            if (!extend_deferred_exec(deferred_bt_pairing, PAIRING_LONG_PRESS)) {
                deferred_bt_pairing = defer_exec(PAIRING_LONG_PRESS, start_pairing, NULL);
            }
            iton_bt_switch_profile(profile);
        } else {
            set_output(OUTPUT_USB);
            kb_config.bt_profile = PROFILE_USB;
        }
        current_profile_state = STATE_CONNECTING;
        if (kb_config.bt_profile != profile) {
            kb_config.bt_profile = profile;
            eeconfig_update_user(kb_config.raw);
        }
    }
}

/**
 * ITON callback methods, setting a local state variable
*/
void iton_bt_connection_successful() {
    current_profile_state = STATE_CONNECTED;
    set_output(OUTPUT_BLUETOOTH);
}

void iton_bt_entered_pairing() {
    current_profile_state = STATE_PAIRING;
};

void iton_bt_disconnected() {
    key_pressed_time = timer_read();
    current_profile_state = STATE_DISCONNECTED;
};

void iton_bt_enters_connection_state() {
    current_profile_state = STATE_CONNECTING;
};


/**
 * Setting led on / off (blinking freq defined in deferred exec blink routine)
*/
bool rgb_matrix_indicators_user(void) {
    if (profile_led) {
        rgb_matrix_set_color(17 + kb_config.bt_profile, 0, 0, 255);
    }

    return true;
}

bool dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 0: // macos/windows toggle
            if(active) { // Mac mode
            } else { // Windows mode
            }
            return false;
        case 1:
            if (active) {
                bluetooth_enabled = false;
                select_profile(PROFILE_USB);
            } else {
                bluetooth_enabled = true;
                if (kb_config.bt_profile == PROFILE_USB) {
                    set_output(OUTPUT_USB);
                } else {
                    set_output(OUTPUT_NONE);
                }
                current_profile_state = STATE_CONNECTING;
                defer_exec(100, led_blinking, NULL);
            }
            return false;
    }
    return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (bluetooth_enabled) {
        if (record->event.pressed) {
            switch (keycode) {
                case KC_BT1:
                    select_profile(PROFILE_BT_1);
                    return false;
                case KC_BT2:
                    select_profile(PROFILE_BT_2);
                    return false;
                case KC_BT3:
                    select_profile(PROFILE_BT_3);
                    return false;
                case KC_USB:
                    select_profile(PROFILE_USB);
                    return false;
                case KC_PAIR:
                    iton_bt_enter_pairing();
                    return false;
            }
        } else if (keycode >= KC_BT1 && keycode <= KC_BT3) {
            cancel_deferred_exec(deferred_bt_pairing);
            return false;
        }
    }
    return true;
}

void keyboard_post_init_user(void) {
    kb_config.raw = eeconfig_read_user();
    if (kb_config.bt_profile >= PROFILE_USB) {
        kb_config.bt_profile = 0;
    }
    //debug_enable=true;
    //debug_matrix=true;
    //debug_keyboard=true;
    //debug_mouse=true;
}


bool caps_word_press_user(uint16_t keycode) {
    switch (keycode) {
        // Keycodes that continue Caps Word, with shift applied.
        case KC_A ... KC_Z:
        case KC_LBRC: // nordic iso layout national key
        case KC_SCLN: // ^^
        case KC_QUOT: // ^^
        case KC_MINS:
        case KC_SLSH: // Nordic iso minus / underscore
            add_weak_mods(MOD_BIT(KC_LSFT));  // Apply shift to next key.
            return true;

        // Keycodes that continue Caps Word, without shifting.
        case KC_1 ... KC_0:
        case KC_BSPC:
        case KC_DEL:
            return true;
        default:
            return false;  // Deactivate Caps Word.
    }
}
