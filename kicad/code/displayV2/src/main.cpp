#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Stepper.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// display is 1301

// The remote service we wish to connect to.
static BLEUUID serviceUUID("ea1ac798-29db-461d-9d49-109d7f01f0dd");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("cd0ac878-6800-4b50-be03-4eef6a25b14f");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

#define BUTTON_PIN D0 // button pin
#define PIXEL_PIN D7  // NeoPixel pin
#define PIXEL_COUNT 1
#define STEPS 315
bool buttonEnabled = false;

int motorPins[4] = {D5, D6, D3, D1};

int inputSignal = 0;
int currentPosition = 0;

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
Stepper myStepper(STEPS, motorPins[0], motorPins[1], motorPins[2], motorPins[3]);

unsigned long lastButtonPress = 0;
bool onState = false; // True means ON state, False means OFF state.

/**
 * Sets the color of the NeoPixel.
 *
 * @param red The intensity of the red color (0-255).
 * @param green The intensity of the green color (0-255).
 * @param blue The intensity of the blue color (0-255).
 * @param duration The duration in milliseconds for which the color should be displayed.
 */
void setNeoPixelColor(uint8_t red, uint8_t green, uint8_t blue, int duration)
{
  strip.setPixelColor(0, strip.Color(red, green, blue));
  strip.show();
  delay(duration);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
}

void toggleColorBasedOnState()
{
  if (onState)
  {
    // If ON state, double click turns the LED green
    setNeoPixelColor(0, 255, 0, 3000); // Green for 5 seconds
  }
  else
  {
    // If OFF state, double click turns the LED red
    setNeoPixelColor(255, 0, 0, 3000); // Red for 5 seconds
  }
}

void sendCommand(const char *command)
{
  if (connected)
  {
    // Assuming 'pRemoteCharacteristic' is correctly set up to point to the server's characteristic
    pRemoteCharacteristic->writeValue(command, strlen(command));
    Serial.println("Command sent: " + String(command));
  }
}

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
  }

  void onDisconnect(BLEClient *pclient)
  {
    connected = false;
    Serial.println("onDisconnect");
  }
};

void moveStepper(int value)
{
  // Invert the mapping so that 0% maps to the left side (0 degrees) and 100% to the right side (315 degrees).
  // This means we subtract the calculated target position from the maximum to invert the direction.
  float targetPositionDegrees = 315 - (315.0 / 100) * value;

  // Calculate the target position in steps.
  // Since 600 steps are equivalent to 315 degrees, we find the proportional number of steps for the target position.
  int targetPositionSteps = (int)((600 / 315.0) * targetPositionDegrees);

  // Calculate the number of steps to move from the current position.
  int stepsToTarget = targetPositionSteps - currentPosition;

  // Move the stepper motor by the calculated steps.
  myStepper.step(stepsToTarget);

  // Update the current position in steps.
  currentPosition = targetPositionSteps;
}

void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
  String value = "";
  for (int i = 0; i < length; i++)
  {
    value += (char)pData[i];
  }
  Serial.print("Received data: ");
  Serial.println(value);

  // Assuming value is directly the parameter for moveStepper
  int sleepQuality = value.toInt();
  moveStepper(sleepQuality);
}

bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice); // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  pClient->setMTU(517); // set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead())
  {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID))
    {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    }
  }
};

void setup()
{
  Serial.begin(9600);
  BLEDevice::init("DISPLAY_CLIENT");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  myStepper.setSpeed(100);
}

void loop()
{
  static uint8_t lastButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;
  uint8_t reading = digitalRead(BUTTON_PIN);
  static bool commandSent = false;
  static bool isFirstClick = false;

  if (doConnect == true)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
    }
    else
    {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
  }

  // Button state has changed
  if (reading == LOW && lastButtonState == HIGH)
  {
    if (!buttonEnabled)
    {
      Serial.println("Button enabled");
      buttonEnabled = true;
      setNeoPixelColor(255, 255, 255, 1000);
      sendCommand("start");
      commandSent = true;
    }
    else
    {

      if ((millis() - lastDebounceTime) > debounceDelay)
      {
        // Button state has changed
        if (reading == LOW && lastButtonState == HIGH)
        {
          if (!buttonEnabled)
          {
            Serial.println("Button enabled");
            buttonEnabled = true;                  // Activate button functionality on first press
            setNeoPixelColor(255, 255, 255, 2000); // Indicate button activation
            sendCommand("start");
            commandSent = true;
          }
          else
          {
            // Single click logic (not a double click) - toggle start/stop
            isFirstClick = false; // Reset for the next action
            if (commandSent)
            {
              buttonEnabled = false;  
              sendCommand("stop");
              commandSent = false; // Reset commandSent for toggling
            }
            else
            {
              sendCommand("start");
              commandSent = true;
            }
          }

          // For subsequent presses, check for double click for LED color toggle
          static unsigned long firstClickTime = 0;
          if (isFirstClick && (millis() - firstClickTime) < 500)
          {
            // Double click detected
            toggleColorBasedOnState();
            isFirstClick = false;
          }
          else if (!isFirstClick)
          {
            isFirstClick = true;
            firstClickTime = millis();
          }
          else
          {
            // Single click logic (not a double click) - toggle start/stop
            isFirstClick = false; // Reset for the next action
            if (commandSent)
            {
              sendCommand("stop");
              commandSent = false; // Reset commandSent for toggling
            }
            else
            {
              sendCommand("start");
              commandSent = true;
            }
          }
        }
      }

      lastButtonState = reading;
    }
    delay(1000);
  }
}