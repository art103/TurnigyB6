/*
    ChARTurn: Custom firmware for Turnigy B6 Compact charger.
    Copyright (C) 2016 Richard Taylor <richard@artaylor.co.uk>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _CHARGER_H_
#define _CHARGER_H_

#define NUM_CHANNELS    (4)     // Number of balance / LED channels.

#define MAX_CELL_V      (4250)  // Charge Abort voltage
#define MAX_CELL_V_CHG  (4170)  // Charge Termination voltage
#define MIN_CELL_V      (2900)  // Minimum voltage to register a cell as usable
#define CELL_PRESENT_V  (1500)  // Value to register a cell as present
#define PACK_PRESENT_V  (2000)  // Value above which to register the main connector present.

#define INITIAL_CHARGE_CURRENT  (1300)       // Measurement charge current (mA)
#define MAX_CHARGE_CURRENT      (5000)      // Maximum current output (mA)
#define MAX_CHARGE_POWER        (50000000)  // Maximum power output (mW)
#define CHARGE_RATE             (1)         // Charge rate in C

#define BALANCE_THRESHOLD       (5)         // Threshold in mV between cells before balancing.

#define BATTERY_SETTLE_TIME     (10)
#define BATTERY_MEASURE_TIME    (60)        // How often to stop balancing and check the capacity.

#define CHARGE_TIMEOUT          (5400000)   // How long to charge before aborting.

#define ENABLE_EXTRA_LCD_INFO               // Extra LCD output (may exceed the flash limit).

// Force a specific charge current and don't measure the pack (debugging).
#define FIXED_CHARGE_CURRENT  (1400)

/*
 * IO Pin Structure
 */
struct _pin {
    GPIO_TypeDef* port;
    GPIO_Pin_TypeDef pin;
};

/*
 * Charger Logic State
 */
typedef enum
{
    STATE_ERROR,
    STATE_CHECKING,
    STATE_SINGLE_CELL,
    STATE_MEASURING,
    STATE_CHARGING,
    STATE_DONE
} State;
extern State state;

/*
 * See main.c for a description of these.
 */
extern volatile uint16_t adc_values[NUM_CHANNELS + 3];
extern uint16_t input_voltage;
extern uint16_t battery_voltage;
extern uint16_t battery_current;

extern uint8_t num_cells;

extern uint16_t pwm_curr;
extern uint16_t pwm_vol;

extern uint16_t average_count;
extern uint32_t batt_curr_avg;
extern uint32_t batt_vol_avg;
extern uint32_t input_vol_avg;
extern uint32_t balance_avg[NUM_CHANNELS];

extern uint16_t cell_min;
extern uint16_t cell_max;

extern uint16_t battery_capacity;

extern uint8_t balancing;
extern uint16_t target_current;

extern const uint16_t calibration[NUM_CHANNELS + 3];

/*
 * Delay function based on system timer
 */
void delay_ms(uint32_t ms);

/*
 * When the LCD is enabled, this formats and displays the cell and
 * pack information on screen.
 */
void update_lcd_info(void);

#endif // _CHARGER_H_
