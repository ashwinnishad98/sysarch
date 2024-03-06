#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "driver/i2s.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <sleepanalysis.h>
#define SERVICE_UUID "ea1ac798-29db-461d-9d49-109d7f01f0dd"
#define CHARACTERISTIC_UUID "cd0ac878-6800-4b50-be03-4eef6a25b14f"
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// I2S pins
#define I2S_WS 3
#define I2S_SCK 1
#define I2S_SD 4
#define SAMPLE_RATE 16000
#define BITS_PER_SAMPLE 16
#define CHANNELS 1
#define RECORD_TIME 60000 // 60 seconds in milliseconds

Adafruit_MPU6050 mpu;

bool lastButton = false;

// High pass filter parameters
float alpha = 0.9;
float prevFilteredX = 0, prevFilteredY = 0, prevFilteredZ = 0;
float prevRawX = 0, prevRawY = 0, prevRawZ = 0;

unsigned long startMillis;
const unsigned long period = 60000;      // 1 minute
const unsigned long awakePeriod = 15000; // 15 seconds
unsigned long awakeStartMillis;
unsigned int awakeCounter = 0;
bool isAwake = false;
bool startCollection = false;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    startCollection = false;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic) override
  {
    std::string value = pCharacteristic->getValue();
    if (value == "start")
    {
      startCollection = true;
      Serial.println("Start command received");
    }
    else if (value == "stop")
    {
      startCollection = false;
      Serial.println("Stop command received");
    }
  }
};

int collectMPUData()
{
  // Read the raw values
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // High pass filter to remove the influence of gravity
  float filteredX = alpha * (prevFilteredX + a.acceleration.x - prevRawX);
  float filteredY = alpha * (prevFilteredY + a.acceleration.y - prevRawY);
  float filteredZ = alpha * (prevFilteredZ + a.acceleration.z - prevRawZ);

  // Update the previous values for the next iteration
  prevFilteredX = filteredX;
  prevFilteredY = filteredY;
  prevFilteredZ = filteredZ;
  prevRawX = a.acceleration.x;
  prevRawY = a.acceleration.y;
  prevRawZ = a.acceleration.z;

  // Calculate the magnitude of the acceleration vector
  float magnitude = sqrt(filteredX * filteredX + filteredY * filteredY + filteredZ * filteredZ);

  // Threshold to determine movement, tune based on your requirements
  float threshold = 0.5; // Example threshold, adjust as needed

  // Detect movement based on the magnitude surpassing the threshold
  if (magnitude > threshold)
  {
    if (!isAwake)
    {
      isAwake = true;
      awakeStartMillis = millis();
    }
  }
  else
  {
    isAwake = false;
  }

  // If there was movement for 15 continuous seconds, increment the counter
  if (isAwake && (millis() - awakeStartMillis >= awakePeriod))
  {
    awakeCounter++;
    isAwake = false; // Reset the awake status
    Serial.println("Movement detected, person is awake.");
  }
  int sleepQuality;
  // Every minute, report the number of times movement was detected
  if (millis() - startMillis >= period)
  {

    switch (awakeCounter)
    {
    case 0:
      sleepQuality = 100;
      break;
    case 1:
      sleepQuality = 75;
      break;
    case 2:
      sleepQuality = 50;
      break;
    case 3:
      sleepQuality = 25;
      break;
    case 4:
      sleepQuality = 0;
      break;
    default:
      sleepQuality = 0; // Handles the case if it's more than 4 for some reason
    }

    Serial.print("One minute passed. Awake count: ");
    Serial.print(awakeCounter);
    Serial.print(", Sleep Quality: ");
    Serial.println(sleepQuality);

    // Reset counter and timer
    awakeCounter = 0;
    startMillis = millis();
  }
  Serial.println("Sleep quality: " + String(sleepQuality));
  return sleepQuality;
}

void collectMicData()
{
  // Buffer to store the received data
  const int buffer_size = SAMPLE_RATE * BITS_PER_SAMPLE / 8 * CHANNELS * 60; // 60 seconds buffer
  char *audio_data = new char[buffer_size];
  size_t bytes_read = 0;

  unsigned long startMillis = millis();
  while (millis() - startMillis < RECORD_TIME)
  {
    // Read data from I2S bus
    i2s_read(I2S_NUM_0, (void *)audio_data, buffer_size, &bytes_read, portMAX_DELAY);
  }

  // At this point, audio_data contains 1 minute of audio, which can be processed or saved.
  delete[] audio_data;

  // Create a signal object for the model
  signal_t signal;
  signal.total_length = sizeof(audio_data) / sizeof(audio_data[0]);
  signal.get_data = &get_audio_signal_data;

  // Perform inference
  ei_impulse_result_t result;
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, true);

  if (res == EI_IMPULSE_OK)
  {
    // Print the classification result
    Serial.print("Label: ");
    Serial.println(result.classification.label);
    Serial.print("Confidence: ");
    Serial.println(result.classification.value);
  }
  else
  {
    Serial.println("Inference failed");
  }
}

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // wait for serial monitor

  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.println("MPU6050 Found!");

  // Configure I2S driver for receiving audio data
  i2s_config_t i2s_config = {
      .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = static_cast<i2s_bits_per_sample_t>(BITS_PER_SAMPLE),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Mono microphone
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 32,
      .dma_buf_len = 64,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  // Pin configuration
  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD};

  // Install and start i2s driver
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  Serial.println("Microphone setup complete!");

  BLEDevice::init("SENSING_SERVER");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Hello World");
  pService->start();
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE setup complete!");
  startMillis = millis(); // initial start time
}

void loop()
{
  // Check if the device is connected and if we should start the collection
  if (deviceConnected && startCollection)
  {
    Serial.println("Start received, start collection");
    int sq = collectMPUData();                      // Start collecting MPU data
    pCharacteristic->setValue(String(sq).c_str());  // Convert int to string to set as value
    pCharacteristic->notify();                      // Notify the client
    Serial.println("Sending value: " + String(sq)); // Debug output
    startCollection = false;                        // Reset startCollection to await next command
    lastButton = true;
  }
  if (deviceConnected && !startCollection && lastButton)
  {
    pCharacteristic->setValue("0"); // Reset the value to 0
    pCharacteristic->notify();      // Notify the client
    lastButton = false;
  }
  {
    Serial.println("Start not received, not collecting");
  }
  // Handle the disconnection
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // Give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // Restart advertising
    Serial.println("Restart advertising");
    oldDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  delay(1000);
}
