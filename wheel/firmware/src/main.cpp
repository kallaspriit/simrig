// https://www.adafruit.com/product/4062
// https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather?view=all
// https://cdn-learn.adafruit.com/downloads/pdf/introducing-the-adafruit-nrf52840-feather.pdf?timestamp=1602412010
// https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather/blehidadafruit
// https://github.com/adafruit/Adafruit_nRF52_Arduino
// https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/libraries/Bluefruit52Lib

// Required buttons
// 1. left arrow
// 2. right arrow
// 3. up arrow
// 4. down arrow
// 5. page left (game based, F3 for Dirt)
// 6. page right (game based, F4 for Dirt)
// 7. enter
// 8. escape
// 9. toggle game
// 10. reset view (F11 for dirt)
// 11. game specific 1
// 12. game specific 2
// 13. game specific 3
// 14. game specific 4

// Dirt game specific buttons
// 1. Propose fixes
// 2. Repair tire
// 3. Window wipers

#include <bluefruit.h>

// battery voltage detection configuration
const float VBAT_MV_PER_LSB = 0.73242188f; // 3000mV/4096 12-bit ADC
const float VBAT_DIVIDER = 0.5f;           // 150K + 150K voltage divider on VBAT
const float VBAT_COMPENSATED_MV_PER_LSB = (1.0f / VBAT_DIVIDER) * VBAT_MV_PER_LSB;

// services
BLEDis deviceInformationService;
BLEHidAdafruit hidService;
BLEBas batteryService;

// pin configuration
const int BUTTON_PIN = 7;
const int CONNECTION_LED_PIN = LED_CONN;

// timing configuration
const unsigned int REPORT_BATTERY_VOLTAGE_INTERVAL_MS = 60000;
const unsigned int INTERACTION_DEEP_SLEEP_DELAY_MS = 30000;
const unsigned int CONNECTION_BLINK_TOGGLE_INTERVAL_MS = 5000;
const unsigned int CONNECTION_BLINK_ON_DURATION_MS = 10;

// runtime info
bool wasButtonPressed = false;
unsigned int lastSendTime = 0;
unsigned int lastReportBatteryVoltageTime = 0;
unsigned int lastInteractionTime = 0;
unsigned int lastConnectionBlinkToggleTime = 0;
int connectionBlinkState = LOW;

void startAdvertising(void)
{
  // advertising as HID service
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);

  // add the hid service
  Bluefruit.Advertising.addService(hidService);

  // also advertise device name
  Bluefruit.Advertising.addName();

  // enable auto advertising if disconnected
  Bluefruit.Advertising.restartOnDisconnect(true);

  // configure intervals for afast and slow modes (in unit of 0.625 ms)
  Bluefruit.Advertising.setInterval(32, 244);

  // configure number of seconds in fast advertising mode
  Bluefruit.Advertising.setFastTimeout(30);

  // start advertising forever
  Bluefruit.Advertising.start(0);
}

/**
  * Callback invoked when received Set LED from central.
  * Must be set previously with setKeyboardLedCallback()
  *
  * The LED bit map is as follows: (also defined by KEYBOARD_LED_*)
  * Kana (4) | Compose (3) | ScrollLock (2) | CapsLock (1) | Numlock (0)
*/
// void setKeyboardLed(uint16_t conn_handle, uint8_t led_bitmap)
// {
//   (void)conn_handle;

//   // light up red led if any bits is set
//   if (led_bitmap)
//   {
//     ledOn(LED_RED);
//   }
//   else
//   {
//     ledOff(LED_RED);
//   }
// }

float getBatteryVoltageMillivolts()
{
  float raw;

  // set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // let the ADC settle
  delay(1);

  // get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(PIN_VBAT);

  // set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // convert the raw value to compensated mv, taking the resistor-divider into
  // account ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * VBAT_COMPENSATED_MV_PER_LSB;
}

int getBatteryRemainingPercentage(float millivolts)
{
  if (millivolts < 3300)
  {
    return 0;
  }

  if (millivolts < 3600)
  {
    millivolts -= 3300;

    return millivolts / 30;
  }

  millivolts -= 3600;

  int percentage = 10 + (millivolts * 0.15F);

  if (percentage > 100)
  {
    return 100;
  }
  else if (percentage < 0)
  {
    return 0;
  }

  return percentage;
}

void goToDeepSleep()
{
  // turn of all power hungry devices
  digitalWrite(CONNECTION_LED_PIN, LOW);

  // put nrf52 to deep sleep, gets waken up when one of the sense inputs is pressed
  sd_power_system_off();
}

void setup()
{
  // setup serial
  Serial.begin(115200);

  // wait for serial to respond
  // while (!Serial)
  // {
  //   delay(10);
  // }

  Serial.println("-- Sim Keyboard --");

  // use button as input that also wakes up the device from deep sleep
  pinMode(BUTTON_PIN, INPUT_PULLUP_SENSE);
  pinMode(CONNECTION_LED_PIN, OUTPUT);

  // configure bluetooth
  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  // Bluefruit.setTxPower(-12); // Check bluefruit.h for supported values
  Bluefruit.setName("Sim Keyboard");

  // disable automatic connection led
  Bluefruit.autoConnLed(false);

  // configure and start device information service
  deviceInformationService.setManufacturer("Stagnation Lab");
  deviceInformationService.setModel("Sim Keyboard v1");
  deviceInformationService.begin();

  // start HID
  hidService.begin();

  // set callback for set LED from central
  // hidService.setKeyboardLedCallback(setKeyboardLed);

  // start battery service
  batteryService.begin();

  // start advertising
  startAdvertising();

  // initialize last interaction time (board goes to sleep after a while)
  lastInteractionTime = millis();
}

void loop()
{
  unsigned int currentTime = millis();

  // report battery voltage at certain interval
  if (currentTime - lastReportBatteryVoltageTime >= REPORT_BATTERY_VOLTAGE_INTERVAL_MS)
  {
    // get battery voltage and percentage
    float batteryVoltageMillivolts = getBatteryVoltageMillivolts();
    int batteryRemainingPercentage = getBatteryRemainingPercentage(batteryVoltageMillivolts);

    // log battery level
    Serial.print("Battery: ");
    Serial.print(batteryVoltageMillivolts / 1000.0f);
    Serial.print("V (");
    Serial.print(batteryRemainingPercentage);
    Serial.println("%)");

    // update remaining battery level
    batteryService.notify(batteryRemainingPercentage);

    lastReportBatteryVoltageTime = currentTime;
  }

  bool isButtonPressed = digitalRead(BUTTON_PIN) == LOW;

  if (isButtonPressed && !wasButtonPressed)
  {
    // hidService.keyPress(HID_KEY_ARROW_RIGHT);

    uint8_t keycodes[6] = {HID_KEY_ARROW_RIGHT, HID_KEY_NONE, HID_KEY_NONE,
                           HID_KEY_NONE, HID_KEY_NONE, HID_KEY_NONE};
    hidService.keyboardReport(0, keycodes);

    // lastSendTime = currentTime;
    Serial.println("button down");

    wasButtonPressed = true;
    lastInteractionTime = currentTime;
  }
  else if (!isButtonPressed && wasButtonPressed)
  {
    hidService.keyRelease();

    Serial.println("button up");

    wasButtonPressed = false;
    lastInteractionTime = currentTime;
  }

  // go to deep sleep if there are no interactions for a while
  if (currentTime - lastInteractionTime >= INTERACTION_DEEP_SLEEP_DELAY_MS)
  {
    Serial.print("no interactions detected for");
    Serial.print(INTERACTION_DEEP_SLEEP_DELAY_MS);
    Serial.println("ms, going to deep sleep until any of the buttons is clicked");
    Serial.flush();

    delay(3000);

    goToDeepSleep();
  }

  // decide connection led state
  int targetConnectionLedState = LOW;

  if (Bluefruit.connected())
  {
    targetConnectionLedState = HIGH;
  }
  else
  {
    unsigned long timeSinceLastBlink = currentTime - lastConnectionBlinkToggleTime;

    // toggle blink led at interval
    if (timeSinceLastBlink >= CONNECTION_BLINK_TOGGLE_INTERVAL_MS)
    {
      lastConnectionBlinkToggleTime = currentTime;
    }

    if (timeSinceLastBlink <= CONNECTION_BLINK_ON_DURATION_MS)
    {
      targetConnectionLedState = HIGH;
    }
    else
    {
      targetConnectionLedState = LOW;
    }
  }

  // only update pin if changed
  if (targetConnectionLedState != connectionBlinkState)
  {
    connectionBlinkState = targetConnectionLedState;

    digitalWrite(CONNECTION_LED_PIN, connectionBlinkState);
  }
}