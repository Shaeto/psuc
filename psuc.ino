#include <avr/power.h>
#include <avr/sleep.h>

/*******************************************************************************
  Pico PSU Control application
  Version: 1.03
  Date: 11-03-2018
  Author: Evgeny Sabelskiy

  This is a simple control application for PICO PSU

  This application is licensed under Creative Commons Attribution-ShareAlike 3.0.
  Unported License.

  Revision  Description
  ========  ===========
  1.03      Added case fan control
  1.02      Fixed gpio shutdown request deassertion
  1.01      Added configuration
  1.00      Initial public release (for arduino mini pro 5v)
*******************************************************************************/

/* TODO:
  1. remove bootloader (requires ISP programmer)
  2. implement watchdog
*/

/* ------------- BEGIN CONFIGURATION ---------------- */
// set it to 1 if you want to get debugging output on serial console (115200)
#define PSUC_DEBUG 0

// set to 1 if you want to set HIGH gpio signal for external hardware
// before power down and wait for confirmation signal
// if 0 power will be turned off immediately
#define PSUC_USE_CONFIRMATION_GPIO 1

// set to 1 if you want to use arduino on-board led
#define PSUC_USE_ONBOARD_LED 1

// set to 1 if you want to use external (case) led
#define PSUC_USE_EXTERNAL_LED 1

// set to 1 if you want to control speed of (case) fan
#define PSUC_CONTROL_FAN_SPEED 1

// set fan duty cycle (default is 50%)
#define PSUC_CONTROL_FAN_DUTY_CYCLE 50

// IMPORTANT!!! this is depends on your ATMEGA->[ATX PS_ON PIN] hardware implementation
// set it to LOW if you connected cpu directly to PS_ON
// set to HIGH if you used NPN transistor switch
#define PSUC_PSON_SIGNAL HIGH

// power down system if user pressed button longer than XX seconds
#define PSUC_EMERGENCY_POWER_DOWN_DELAY 10

// return to sleep mode if gpio event didn't occur after XX seconds
#define PSUC_RETURN_TO_POWERON_SLEEP_IN 60
/* -------------- END CONFIGURATION ----------------- */

/* -------------- BEGIN PIN MAPPING ----------------- */
// input (irq): external "power on/off" (momentary) button connected to ground
#define PSUC_POWER_BUTTON_PIN 2

// input (irq): external request for immediate PSU shutdown
#define PSUC_PSU_SHURDOWN_PIN 3

// output: connected to ATX-24 PS_ON (16) pin
#define PSUC_ATX_PSU_ON_PIN 4

// output: connected to external GPIO
// HIGH state is signal for PSU clients to close their business and set HIGH on PSUC_PSU_SHURDOWN_PIN
// for power down
#define PSUC_GPIO_REQUEST_PIN 5

// arduino onboard led
#define PSUC_ARDUINO_LED_PIN LED_BUILTIN

// external case led
#define PSUC_EXTERNAL_LED_PIN 6

// case fan, use pin 11 (asynchronous clock that is not stopped with SLEEP_MODE_PWR_SAVE)
// change timer configuration if you want to remap this pin
#define PSUC_CONTROL_FAN_PIN 11
/* -------------- END PIN MAPPING ------------------- */

enum t_psuc_state {
  // PSUC is initializing
  PSUC_INIT,
  // application is sleeping and waiting for external interrupt, PSU is OFF
  PSUC_SLEEP_OFF,
  // application is sleeping and waiting for external interrupt, PSU is ON
  PSUC_SLEEP_ON,
  // processing turn PSU OFF
  PSUC_POWER_OFF,
  // power off requested by user
  PSUC_EMERGENCY_POWER_OFF,
  // processing turn PSU ON
  PSUC_POWER_ON,
#if PSUC_USE_CONFIRMATION_GPIO
  // read GPIO signals
  PSUC_WAIT_GPIO_READ,
  // confirmate HIGH signal on GPIO and shutdown PSU
  PSUC_WAIT_GPIO_CONFIRMATION
#endif
};

static volatile t_psuc_state psuc_state;
byte power_btn_is_pressed;
byte poweroff_signal_is_received;
#if PSUC_USE_CONFIRMATION_GPIO
byte gpio_request_sent;
unsigned long last_time_wait_alone;
unsigned long start_time_wait_gpio;
#endif

static void change_state(t_psuc_state new_state);
static void reset_signals();

static void gpio_request_shutdown(bool on);

static void power_button_rq();
static void power_down_rq();

static void atx_poweron();
static void atx_poweroff();

static void read_debounced_signals();

static void case_led_ctrl(byte state);
static void led_control(t_psuc_state state);
static void psu_control(t_psuc_state state);

#if PSUC_CONTROL_FAN_SPEED
static void case_fan_speed(int duty_cycle);
#endif

void setup() {
  // turn off psu at startup, not very good idea if we'll need to implement watchdog
  digitalWrite(PSUC_ATX_PSU_ON_PIN, PSUC_PSON_SIGNAL ^ HIGH);
  // we set pin to HIGH before mode change to prevent short-time PS-ON peak
  pinMode(PSUC_ATX_PSU_ON_PIN, OUTPUT);

  // configure power button pin mode to detect LOW signal
  pinMode(PSUC_POWER_BUTTON_PIN, INPUT_PULLUP);

#if PSUC_USE_CONFIRMATION_GPIO
  // configure shut down pin to detect HIGH signal
  pinMode(PSUC_PSU_SHURDOWN_PIN, INPUT);

  // configure gpio request pin
  pinMode(PSUC_GPIO_REQUEST_PIN, OUTPUT);
  digitalWrite(PSUC_GPIO_REQUEST_PIN, LOW);
#endif

  // configure PSUC leds
#if PSUC_USE_ONBOARD_LED
  pinMode(PSUC_ARDUINO_LED_PIN, OUTPUT);
  digitalWrite(PSUC_ARDUINO_LED_PIN, LOW);
#endif

#if PSUC_USE_EXTERNAL_LED
  pinMode(PSUC_EXTERNAL_LED_PIN, OUTPUT);
  digitalWrite(PSUC_EXTERNAL_LED_PIN, LOW);
#endif

#if PSUC_CONTROL_FAN_SPEED
  pinMode(PSUC_CONTROL_FAN_PIN, OUTPUT);
  // set timer 2 divisor to 8 for (Fast) PWM frequency of 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001;
#endif

  // turn off some parts of cpu to lower power consumption
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();

  // set sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  // enable sleep bit
  sleep_enable();

#if PSUC_USE_CONFIRMATION_GPIO
  gpio_request_sent = 0;
#endif
  reset_signals();

#if PSUC_DEBUG
  Serial.begin(115200);
#endif

  // set application initial state
  psuc_state = PSUC_INIT;
  change_state(PSUC_SLEEP_OFF);
}

void atx_poweroff()
{
  // turn off PSU module
  digitalWrite(PSUC_ATX_PSU_ON_PIN, PSUC_PSON_SIGNAL ^ HIGH);
#if PSUC_DEBUG
  Serial.println("atx psu power off");
#endif
}

void atx_poweron()
{
  // turn on PSU module
  digitalWrite(PSUC_ATX_PSU_ON_PIN, PSUC_PSON_SIGNAL);
#if PSUC_DEBUG
  Serial.println("atx psu power on");
#endif
}

#if PSUC_CONTROL_FAN_SPEED
void case_fan_speed(int duty_cycle)
{
  analogWrite(PSUC_CONTROL_FAN_PIN, ((long) 255 * duty_cycle) / 100);
}
#endif

void gpio_request_shutdown(bool on)
{
  if (on) {
    // simulate a short edge and leave signal at HIGH level
    digitalWrite(PSUC_GPIO_REQUEST_PIN, LOW);
    delay(10);
    digitalWrite(PSUC_GPIO_REQUEST_PIN, HIGH);
#if PSUC_DEBUG
    Serial.println("gpio request for shutdown");
#endif
  } else {
    digitalWrite(PSUC_GPIO_REQUEST_PIN, LOW);
#if PSUC_DEBUG
    Serial.println("deassert gpio request");
#endif
  }
}

void power_button_rq() {
  // button is pressed
}

void power_down_rq() {
  // external request for immediate psu down is received
}

void change_state(t_psuc_state new_state)
{
  switch (new_state) {
    case PSUC_SLEEP_OFF:
      // reset current input signals states
      reset_signals();
#if PSUC_DEBUG
      Serial.println("PSUC_SLEEP_OFF");
#endif
      break;
    case PSUC_SLEEP_ON:
      // reset current input signals states
      reset_signals();
#if PSUC_DEBUG
      Serial.println("PSUC_SLEEP_ON");
#endif
      break;
    case PSUC_POWER_OFF:
#if PSUC_DEBUG
      Serial.println("PSUC_POWER_OFF");
#endif
      break;
    case PSUC_EMERGENCY_POWER_OFF:
#if PSUC_DEBUG
      Serial.println("PSUC_EMERGENCY_POWER_OFF");
#endif
      break;
    case PSUC_POWER_ON:
#if PSUC_DEBUG
      Serial.println("PSUC_POWER_ON");
#endif
      break;
#if PSUC_USE_CONFIRMATION_GPIO
    case PSUC_WAIT_GPIO_READ:
      if (psuc_state != PSUC_WAIT_GPIO_CONFIRMATION) {
        last_time_wait_alone = start_time_wait_gpio = millis();
      }
      // reset current input signals states
      reset_signals();
      break;
#endif
    default:
      break;
  }
  psuc_state = new_state;
}

void reset_signals()
{
  power_btn_is_pressed = 0;
  poweroff_signal_is_received = 0;
}

#define PSUC_DEBOUNCE_TIME_MS 100

void read_debounced_signals()
{
  byte s1 = 0, s2 = 0;
  // read signals states during next XX ms (simplest debounce)
  for (int t = 0; t < PSUC_DEBOUNCE_TIME_MS; t++) {
    if (digitalRead(PSUC_PSU_SHURDOWN_PIN) == HIGH)
      s1++;
    if (digitalRead(PSUC_POWER_BUTTON_PIN) == LOW)
      s2++;
    delay(1);
  }

  if (s1 >= (PSUC_DEBOUNCE_TIME_MS - PSUC_DEBOUNCE_TIME_MS / 10))
    poweroff_signal_is_received++;

  if (s2 >= (PSUC_DEBOUNCE_TIME_MS - PSUC_DEBOUNCE_TIME_MS / 10))
    power_btn_is_pressed++;
}

void case_led_ctrl(byte state)
{
  if (state) {
#if PSUC_USE_EXTERNAL_LED
    digitalWrite(PSUC_EXTERNAL_LED_PIN, HIGH);
#endif
#if PSUC_USE_ONBOARD_LED
    digitalWrite(PSUC_ARDUINO_LED_PIN, HIGH);
#endif
  } else {
#if PSUC_USE_EXTERNAL_LED
    digitalWrite(PSUC_EXTERNAL_LED_PIN, LOW);
#endif
#if PSUC_USE_ONBOARD_LED
    digitalWrite(PSUC_ARDUINO_LED_PIN, LOW);
#endif
  }
}

void led_control(t_psuc_state state)
{
  switch (state) {
    case PSUC_SLEEP_OFF:
      case_led_ctrl(LOW);
      break;
    case PSUC_SLEEP_ON:
      case_led_ctrl(HIGH);
      break;
    case PSUC_POWER_OFF:
      break;
    case PSUC_POWER_ON:
      break;
#if PSUC_USE_CONFIRMATION_GPIO
    case PSUC_WAIT_GPIO_READ:
    case PSUC_WAIT_GPIO_CONFIRMATION:
      case_led_ctrl((millis() / 500) % 2 ? HIGH : LOW);
      break;
#endif
    default:
      break;
  }
}

void psu_control(t_psuc_state state)
{
  switch (state) {
    case PSUC_SLEEP_OFF:
      // power is off, psuc is waiting for command
      // attach interrupt handler to power button pin
      attachInterrupt(digitalPinToInterrupt(PSUC_POWER_BUTTON_PIN), power_button_rq, LOW);
#if 0
      // not sure if we need this in power off state
#if PSUC_USE_CONFIRMATION_GPIO
      // attach interrupt handler to power off request pin
      attachInterrupt(digitalPinToInterrupt(PSUC_PSU_SHURDOWN_PIN), power_down_rq, HIGH);
#endif
#endif
      // enable sleep bit
      sleep_enable();

#if PSUC_DEBUG
      Serial.end();
#endif
      // going to sleep
      sleep_mode();

      // first thing after waking from sleep
      sleep_disable();

      // disable interrupts
      detachInterrupt(digitalPinToInterrupt(PSUC_POWER_BUTTON_PIN));
#if 0
#if PSUC_USE_CONFIRMATION_GPIO
      detachInterrupt(digitalPinToInterrupt(PSUC_PSU_SHURDOWN_PIN));
#endif
#endif

#if PSUC_DEBUG
      Serial.begin(115200);
#endif
      read_debounced_signals();

      if (power_btn_is_pressed)
        change_state(PSUC_POWER_ON);
      break;
    case PSUC_SLEEP_ON:
      // power is on, waiting for command

      // attach interrupt handler to power button pin
      attachInterrupt(digitalPinToInterrupt(PSUC_POWER_BUTTON_PIN), power_button_rq, LOW);
#if PSUC_USE_CONFIRMATION_GPIO
      // attach interrupt handler to power off request pin
      attachInterrupt(digitalPinToInterrupt(PSUC_PSU_SHURDOWN_PIN), power_down_rq, HIGH);
#endif
      // enable sleep bit
      sleep_enable();

#if PSUC_DEBUG
      Serial.end();
#endif
      // going to sleep
      sleep_mode();

      // first thing after waking from sleep
      sleep_disable();

      // disable interrupts
      detachInterrupt(digitalPinToInterrupt(PSUC_POWER_BUTTON_PIN));
#if PSUC_USE_CONFIRMATION_GPIO
      detachInterrupt(digitalPinToInterrupt(PSUC_PSU_SHURDOWN_PIN));
#endif
#if PSUC_DEBUG
      Serial.begin(115200);
#endif
      read_debounced_signals();

#if PSUC_USE_CONFIRMATION_GPIO
      if (poweroff_signal_is_received) {
#if PSUC_DEBUG
        Serial.print("gpio rq ");
        Serial.println(poweroff_signal_is_received);
#endif
        change_state(PSUC_POWER_OFF);
      } else
#endif
        if (power_btn_is_pressed) {
#if PSUC_USE_CONFIRMATION_GPIO
          gpio_request_shutdown(true);
          change_state(PSUC_WAIT_GPIO_READ);
#else
          // confirmation gpio is disabled by config, power off immediately
          change_state(PSUC_POWER_OFF);
#endif
        } else {
#if PSUC_DEBUG
          Serial.println("bogus interrupt ?");
#endif
          // stay in power-on sleep mode
        }
      break;
    case PSUC_POWER_OFF:
      atx_poweroff();
      delay(1000);
      change_state(PSUC_SLEEP_OFF);
#if PSUC_CONTROL_FAN_SPEED
      case_fan_speed(0);
#endif
      break;
    case PSUC_EMERGENCY_POWER_OFF:
      change_state(PSUC_POWER_OFF);
      break;
    case PSUC_POWER_ON:
      atx_poweron();
      delay(1000);
#if PSUC_CONTROL_FAN_SPEED
      case_fan_speed(PSUC_CONTROL_FAN_DUTY_CYCLE);
#endif
      change_state(PSUC_SLEEP_ON);
      break;
#if PSUC_USE_CONFIRMATION_GPIO
    case PSUC_WAIT_GPIO_READ:
      read_debounced_signals();
      change_state(PSUC_WAIT_GPIO_CONFIRMATION);
      break;
    case PSUC_WAIT_GPIO_CONFIRMATION:
      if (poweroff_signal_is_received) {
#if PSUC_DEBUG
        Serial.println("POWER OFF IS CONFIRMED");
#endif
        gpio_request_shutdown(false);
        change_state(PSUC_POWER_OFF);
      } else if (power_btn_is_pressed) {
        change_state(PSUC_WAIT_GPIO_READ);
      } else {
        unsigned long time_alone = abs(millis() - last_time_wait_alone) / 1000;
        if (time_alone) {
          // user released the button
          if (time_alone > PSUC_EMERGENCY_POWER_DOWN_DELAY) {
            // user pressed button > XX seconds
            // emergency case power down !
            gpio_request_shutdown(false);
            change_state(PSUC_EMERGENCY_POWER_OFF);
          } else {
            // return to normal power on sleep
            gpio_request_shutdown(false);
            change_state(PSUC_SLEEP_ON);
          }
        } else {
          unsigned long time_wait = abs(millis() - start_time_wait_gpio) / 1000;
          if (time_wait >= PSUC_RETURN_TO_POWERON_SLEEP_IN) {
            // shutdown is not confirmed, return to power on state
            gpio_request_shutdown(false);
            change_state(PSUC_SLEEP_ON);
          } else
            change_state(PSUC_WAIT_GPIO_READ);
        }

        last_time_wait_alone = millis();
      }
      break;
#endif
    default:
      break;
  }
}

void loop() {
  led_control(psuc_state);
  psu_control(psuc_state);
  delay(10);
}

