#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include "BLEDevice.h"

#define BUTTON_PIN A0 // button pin
#define PIXEL_PIN 5
#define PIXEL_COUNT 1
int motorPins[4] = {A1, A2, A3, SDA};

Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

BluetoothSerial SerialBT;
bool buttonState = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
bool lastButtonState = LOW;
bool currentButtonState = LOW;
int clickCount = 0;
unsigned long lastClickTime = 0;
bool flagSingleClick = false;
bool booleanValue = 0;

// TODO
// The remote service we wish to connect to.
static BLEUUID serviceUUID("");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("");
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

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

// TODO
static void notifyCallback( 
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{
  Serial.println();
}

bool connectToServer()
{
  Serial.print("Forming a connection to: ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println("Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);
  Serial.println("Connected to server");
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
  Serial.println("Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println("Found our characteristic");

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

/**
 * Moves the stepper motor.
 */
void moveStepperMotor()
{
  static bool direction = false;
  // Step through the motor sequence
  for (int i = 0; i < 600; i++)
  { // Adjust step count as needed
    for (int pin = 0; pin < 4; pin++)
    {
      digitalWrite(motorPins[pin], HIGH);
      delay(2);
      digitalWrite(motorPins[pin], LOW);
    }
  }

  // Reverse the motor direction by reversing the pin order
  if (direction)
  {
    int temp = motorPins[0];
    motorPins[0] = motorPins[3];
    motorPins[3] = motorPins[2];
    motorPins[2] = motorPins[1];
    motorPins[1] = temp;
  }
  else
  {
    int temp = motorPins[3];
    motorPins[3] = motorPins[0];
    motorPins[0] = motorPins[1];
    motorPins[1] = motorPins[2];
    motorPins[2] = temp;
  }
  direction = !direction;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void setup()
{
  Serial.begin(9600);
  SerialBT.begin("ESP32_BT_DISPLAY_514"); // Bluetooth device name

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (int i = 0; i < 4; i++)
  {
    pinMode(motorPins[i], OUTPUT);
  }

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop()
{
  int reading = digitalRead(BUTTON_PIN);

  // if the switch changed, due to noise or pressing:
  if (reading != lastButtonState)
  {
    // reset  debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    // if button state changed:
    if (reading != buttonState)
    {
      buttonState = reading;
      // toggle if new button state is HIGH
      if (buttonState == HIGH)
      {
        clickCount++;
        lastClickTime = millis();
      }
    }
  }

  // single click
  if (clickCount == 1 && (millis() - lastClickTime) > 200)
  { // 200ms for single click detection
    flagSingleClick = true;
    clickCount = 0;
  }

  // double click
  if (clickCount == 2 && (millis() - lastClickTime) < 200)
  { // within 200ms for double click
    if (booleanValue == 1)
    {
      setNeoPixelColor(0, 255, 0, 5000); // Green for 5 seconds
    }
    else
    {
      setNeoPixelColor(255, 0, 0, 5000); // Red for 5 seconds
    }
    clickCount = 0;
  }

  if (flagSingleClick)
  {
    // Send boolean 1 over Bluetooth i.e. sleep tracking is active
    SerialBT.write(1);
    setNeoPixelColor(255, 255, 255, 3000); // White for 3 seconds
    booleanValue = 1;
    flagSingleClick = false;
  }

  // Read from Bluetooth
  if (SerialBT.available())
  {
    int received = SerialBT.read();
    if (received == 0)
    {
      moveStepperMotor();
      booleanValue = 0;
    }
  }

  lastButtonState = reading;
}
