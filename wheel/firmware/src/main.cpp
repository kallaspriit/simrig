// https://www.adafruit.com/product/4062
// https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather?view=all
// https://cdn-learn.adafruit.com/downloads/pdf/introducing-the-adafruit-nrf52840-feather.pdf?timestamp=1602412010
// https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather/blehidadafruit
// https://github.com/adafruit/Adafruit_nRF52_Arduino
// https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/libraries/Bluefruit52Lib
// https://w3c.github.io/uievents/tools/key-event-viewer.html

#include <bluefruit.h>

// battery voltage detection configuration
const float VBAT_MV_PER_LSB = 0.73242188f; // 3000mV/4096 12-bit ADC
const float VBAT_DIVIDER = 0.5f;           // 150K + 150K voltage divider on VBAT
const float VBAT_COMPENSATED_MV_PER_LSB = (1.0f / VBAT_DIVIDER) * VBAT_MV_PER_LSB;

// button pins mapping (A1 and A2 dont seem to work, auto wake up)
const int BUTTON_ENTER_PIN = 13;            // yellow
const int BUTTON_ESCAPE_PIN = 5;            // yellow
const int BUTTON_TOP_RIGHT_OUTER_PIN = A2;  // red
const int BUTTON_TOP_RIGHT_MIDDLE_PIN = A3; // green
const int BUTTON_TOP_RIGHT_INNER_PIN = A4;  // yellow [choose game]
const int BUTTON_TOP_LEFT_OUTER_PIN = A5;   // red
const int BUTTON_TOP_LEFT_MIDDLE_PIN = SCK; // green
const int BUTTON_TOP_LEFT_INNER_PIN = MOSI; // yellow
const int BUTTON_ARROW_RIGHT_PIN = MISO;    // yellow
const int BUTTON_ARROW_LEFT_PIN = 6;        // blue
const int BUTTON_ARROW_UP_PIN = 9;          // white
const int BUTTON_ARROW_DOWN_PIN = 10;       // green
const int BUTTON_PAGE_NEXT_PIN = 11;        // black
const int BUTTON_PAGE_PREVIOUS_PIN = 12;    // red

// pin to use for choosing which game mapping to use
const int CHOOSE_GAME_PIN = BUTTON_TOP_RIGHT_INNER_PIN;

// pin to use to show bluetooth connection status
const int CONNECTION_LED_PIN = LED_CONN;

// number of games we have mappings for
const int GAME_COUNT = 2;

// number of mapped buttons
const int BUTTON_COUNT = 13;

// button pins (mapping below matches button order)
const byte BUTTON_PINS[BUTTON_COUNT] = {
    // bottom right enter and escape
    BUTTON_ENTER_PIN,
    BUTTON_ESCAPE_PIN,

    // bottom left arrow buttons
    BUTTON_ARROW_RIGHT_PIN,
    BUTTON_ARROW_LEFT_PIN,
    BUTTON_ARROW_UP_PIN,
    BUTTON_ARROW_DOWN_PIN,

    // bottom left page buttons
    BUTTON_PAGE_PREVIOUS_PIN,
    BUTTON_PAGE_NEXT_PIN,

    // top left buttons
    BUTTON_TOP_LEFT_OUTER_PIN,
    BUTTON_TOP_LEFT_MIDDLE_PIN,
    BUTTON_TOP_LEFT_INNER_PIN,

    // top right buttons (top right inner is used to toggle between games)
    BUTTON_TOP_RIGHT_MIDDLE_PIN,
    BUTTON_TOP_RIGHT_OUTER_PIN,
};

// mapping of button index to action based on selected game
uint8_t buttonMapping[GAME_COUNT][BUTTON_COUNT] = {
    // Dirt Rally 2.0
    {
        HID_KEY_RETURN,
        HID_KEY_ESCAPE,

        HID_KEY_ARROW_RIGHT,
        HID_KEY_ARROW_LEFT,
        HID_KEY_ARROW_UP,
        HID_KEY_ARROW_DOWN,

        HID_KEY_F3, // page left
        HID_KEY_F4, // page right

        HID_KEY_W,   // wipers
        HID_KEY_C,   // camera
        HID_KEY_F11, // reset VR view

        HID_KEY_F2, // fix/propose
        HID_KEY_H,  // headlights
    },
    // WRC 9
    {
        HID_KEY_RETURN,
        HID_KEY_ESCAPE,

        HID_KEY_ARROW_RIGHT,
        HID_KEY_ARROW_LEFT,
        HID_KEY_ARROW_UP,
        HID_KEY_ARROW_DOWN,

        HID_KEY_1, // page left
        HID_KEY_3, // page right

        HID_KEY_W,   // wipers
        HID_KEY_C,   // camera
        HID_KEY_F11, // reset VR view

        HID_KEY_F12, // what else would we need?
        HID_KEY_H,   // headlights
    },
};

// timing configuration
const unsigned int REPORT_BATTERY_VOLTAGE_INTERVAL_MS = 60000; // report battery every minute
const unsigned int INTERACTION_DEEP_SLEEP_DELAY_MS = 300000;   // go to sleep after 5 minutes of inactivity
const unsigned int CONNECTION_BLINK_TOGGLE_INTERVAL_MS = 5000; // how often to blink if not connected
const unsigned int CONNECTION_BLINK_ON_DURATION_MS = 10;       // how long to show led when blinking
const unsigned int REPORT_BUTTONS_CHANGED_INTERVAL_MS = 100;   // minimum interval at which to check/report button presses

// runtime info
unsigned int lastButtonsChangedTime = 0;
unsigned int lastReportBatteryVoltageTime = 0;
unsigned int lastConnectionBlinkToggleTime = 0;
int lastPressedButtonCount = 0;
int connectionBlinkState = LOW;
int activeGameIndex = 0;

// services
BLEDis deviceInformationService;
BLEHidAdafruit hidService;
BLEBas batteryService;

// starts bluetooth advertising
void startAdvertising()
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

// returns current battery voltage in millivolts
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

// return how much battery is remaining given battery voltage in millivolts
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

// puts the board to deep sleep to preserve battery
void goToDeepSleep()
{
  // turn of all power hungry devices
  digitalWrite(CONNECTION_LED_PIN, LOW);

  // put nrf52 to deep sleep, gets waken up when one of the sense inputs is pressed
  sd_power_system_off();
}

// called once on initial setup
void setup()
{
  // setup serial
  Serial.begin(115200);

  Serial.println("-- Sim Keyboard --");

  // setup pins
  pinMode(CONNECTION_LED_PIN, OUTPUT);
  pinMode(CHOOSE_GAME_PIN, INPUT_PULLUP_SENSE);

  // setup button pins as inputs with pullup and wake-up sensing
  for (int i = 0; i < BUTTON_COUNT; i++)
  {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP_SENSE);
  }

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

  // start battery service
  batteryService.begin();

  // start advertising
  startAdvertising();

  // initialize last buttons changed time (board goes to sleep after a while)
  lastButtonsChangedTime = millis();
}

// main loop, called continuosly as fast as possible
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

  // consider button changes at certain interval
  if (currentTime - lastButtonsChangedTime >= REPORT_BUTTONS_CHANGED_INTERVAL_MS)
  {
    // check whether changing game is requested
    bool isChooseGameDown = digitalRead(CHOOSE_GAME_PIN) == LOW;

    // handle choosing game
    if (isChooseGameDown)
    {
      activeGameIndex = (activeGameIndex + 1) % GAME_COUNT;

      Serial.print("Switched to game ");
      Serial.print(activeGameIndex + 1);
      Serial.print(" of ");
      Serial.println(GAME_COUNT);

      lastButtonsChangedTime = currentTime;
    }
    else
    {
      // handle pressing any of the buttons
      uint8_t keyCodes[] = {HID_KEY_NONE,
                            HID_KEY_NONE,
                            HID_KEY_NONE,
                            HID_KEY_NONE,
                            HID_KEY_NONE,
                            HID_KEY_NONE};
      int pressedButtonCount = 0;

      for (int i = 0; i < BUTTON_COUNT; i++)
      {
        // bool isButtonDown = Buttons.down(i);
        bool isButtonDown = digitalRead(BUTTON_PINS[i]) == LOW;

        // skip if button is not down
        if (!isButtonDown)
        {
          continue;
        }

        // get pressed button key code
        uint8_t keyCode = buttonMapping[activeGameIndex][i];
        int keyCodeIndex = pressedButtonCount++;

        // Serial.print("Button #");
        // Serial.print(i);
        // Serial.print(" is down, adding key code ");
        // Serial.print(keyCode);
        // Serial.print(" (");
        // Serial.print(pressedButtonCount);
        // Serial.println(")");

        // append to list of pressed buttons
        keyCodes[keyCodeIndex] = keyCode;

        // break loop if maximum number of buttons are already pressed
        if (pressedButtonCount == 6)
        {
          break;
        }
      }

      // report pressed buttons or release if number of pressed buttons changes
      if (pressedButtonCount != lastPressedButtonCount)
      {
        if (pressedButtonCount > 0)
        {
          Serial.print("Reporting ");
          Serial.print(pressedButtonCount);
          Serial.println(" buttons");

          hidService.keyboardReport(0, keyCodes);
        }
        else
        {
          Serial.println("Releasing buttons");

          hidService.keyRelease();
        }

        lastPressedButtonCount = pressedButtonCount;
      }

      lastButtonsChangedTime = currentTime;
    }
  }

  // go to deep sleep if there are no interactions for a while
  if (currentTime - lastButtonsChangedTime >= INTERACTION_DEEP_SLEEP_DELAY_MS)
  {
    Serial.print("no interactions detected for ");
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

  // only update connection led state if status has changed
  if (targetConnectionLedState != connectionBlinkState)
  {
    connectionBlinkState = targetConnectionLedState;

    digitalWrite(CONNECTION_LED_PIN, connectionBlinkState);
  }
}