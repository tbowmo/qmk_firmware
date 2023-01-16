/* Copyright 2022 Philip Mourdjis <philip.j.m@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "k6.h"
#ifdef BLUETOOTH_ENABLE
#include "outputselect.h"
#include "iton_bt.h"
#endif

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if (record->event.pressed) {
        switch(keycode) {
#ifdef BLUETOOTH_ENABLE
            case BT_PROFILE1:
                iton_bt_switch_profile(0);
                break;
            case BT_PROFILE2:
                iton_bt_switch_profile(1);
                break;
            case BT_PROFILE3:
                iton_bt_switch_profile(2);
                break;
            case BT_PAIR:
                iton_bt_enter_pairing();
                break;
            case BT_RESET:
                iton_bt_reset_pairing();
                break;
#endif
            default:
                break;
        }
    }
    return process_record_user(keycode, record);
}
