/**
 * Custom SIM racing buttons wheel firmware.
 * 
 * It's basically a custom bluetooth low-energy wireless keyboard with the buttons mapped
 * to specific keycodes based on chosen game.
 * 
 * One button changes the game mapping to use.
 * 
 * Uses Adafruit nRF52840 Feather Express board with built-in Bluetooth LE and Lipo charger.
 * 
 * The onboard LED is used to show connection state:
 * - constantly on: the board is waiting for binding
 * - blinking slowly: successfully paired and awake
 * - off: the board is in deep sleep
 * 
 * @author Priit Kallas <kallaspriit@gmail.com> 01.2021
 */

// possibly helpful links
// https://www.adafruit.com/product/4062
// https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather?view=all
// https://github.com/adafruit/Adafruit_nRF52_Arduino
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
// connection led is constantly on when pairing and blinks at interval if connected (to preserve battery)
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
const unsigned long REPORT_BATTERY_VOLTAGE_INTERVAL_MS = 60000; // report battery every minute
const unsigned long INTERACTION_DEEP_SLEEP_DELAY_MS = 30000;    // go to sleep after 30 seconds of inactivity
const unsigned long CONNECTED_BLINK_INTERVAL_MS = 10000;        // how often to blink if connected
const unsigned long CONNECTING_BLINK_INTERVAL_MS = 1000;        // how often to blink if not connected
const unsigned long CONNECTION_GIVE_UP_DURATION_MS = 30000;     // how long to attempt to connect to a host before giving up and goind to sleep
const unsigned long CONNECTION_BLINK_ON_DURATION_MS = 10;       // how long to show led when blinking
const unsigned long REPORT_BUTTONS_CHANGED_INTERVAL_MS = 100;   // minimum interval at which to check/report button presses

// runtime info
unsigned long lastButtonPressTime = 0;
unsigned long lastReportBatteryTime = 0;
unsigned long lastConnectionBlinkToggleTime = 0;
unsigned long lastConnectedTime = 0;
int lastPressedButtonCount = 0;
int connectionBlinkState = LOW;
int activeGameIndex = 0;
bool wasConnected = false;
bool haveReportedBattery = false;

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
  // notify about going to sleep with led on for a second, then turn it off
  digitalWrite(CONNECTION_LED_PIN, HIGH);
  delay(1000);
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
  lastButtonPressTime = millis();
}

// main loop, called continuosly as fast as possible
void loop()
{
  unsigned int currentTime = millis();

  // detect connected state
  bool isConnected = Bluefruit.connected() > 0;
  bool connectionEstablished = isConnected && !wasConnected;
  bool connectionLost = !isConnected && wasConnected;

  // update was connected state
  wasConnected = isConnected;

  // update last connected time
  if (isConnected)
  {
    lastConnectedTime = currentTime;
  }

  // log connection established / lost
  if (connectionEstablished)
  {
    Serial.println("Connection established");

    // avoid quickly going to sleep
    lastButtonPressTime = currentTime;
  }
  else if (connectionLost)
  {
    Serial.println("Connection lost");

    // make sure to report battery again
    haveReportedBattery = false;
  }

  // calculate time since buttons were last pressed
  unsigned long timeSinceLastButtonPress = currentTime - lastButtonPressTime;

  // go to deep sleep if there are no interactions for a while
  if (isConnected && timeSinceLastButtonPress >= INTERACTION_DEEP_SLEEP_DELAY_MS)
  {
    Serial.print("No interactions detected for ");
    Serial.print(INTERACTION_DEEP_SLEEP_DELAY_MS);
    Serial.println("ms, going to deep sleep until any of the buttons is clicked");

    goToDeepSleep();
  }

  // calculate time since last connected
  unsigned long timeSinceLastConnected = currentTime - lastConnectedTime;

  // go to sleep if have been attempting to establish connection for too long
  if (!isConnected && timeSinceLastConnected >= CONNECTION_GIVE_UP_DURATION_MS)
  {
    Serial.print("Failed to establish bluetooth connection in ");
    Serial.print(CONNECTION_GIVE_UP_DURATION_MS);
    Serial.println("ms, going to deep sleep until any of the buttons is clicked");

    goToDeepSleep();
  }

  // calculate time since last reported battery voltage
  unsigned long timeSinceLastReportedBattery = currentTime - lastReportBatteryTime;

  // report battery voltage if connected at certain interval or if have never reported
  if (isConnected && (!haveReportedBattery || timeSinceLastReportedBattery >= REPORT_BATTERY_VOLTAGE_INTERVAL_MS))
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

    lastReportBatteryTime = currentTime;
    haveReportedBattery = true;
  }

  // consider button changes at certain interval (only if connected)
  if (isConnected && timeSinceLastButtonPress >= REPORT_BUTTONS_CHANGED_INTERVAL_MS)
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

      lastButtonPressTime = currentTime;
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

      // update last buttons changed time if any of the buttons was pressed
      if (pressedButtonCount > 0)
      {
        lastButtonPressTime = currentTime;
      }
    }
  }

  // decide connection led state
  int targetConnectionLedState = LOW;

  // decide connection led blink interval (blinks faster if connecting)
  unsigned long blinkInterval = isConnected ? CONNECTED_BLINK_INTERVAL_MS : CONNECTING_BLINK_INTERVAL_MS;
  unsigned long timeSinceLastBlink = currentTime - lastConnectionBlinkToggleTime;

  // toggle blink led at interval
  if (timeSinceLastBlink >= blinkInterval)
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

  // only update connection led state if status has changed
  if (targetConnectionLedState != connectionBlinkState)
  {
    // log connecting duration
    if (!isConnected)
    {
      Serial.print("Connecting ");
      Serial.print(timeSinceLastConnected);
      Serial.print("/");
      Serial.print(CONNECTION_GIVE_UP_DURATION_MS);
      Serial.println("ms");
    }

    connectionBlinkState = targetConnectionLedState;

    digitalWrite(CONNECTION_LED_PIN, connectionBlinkState);
  }
}