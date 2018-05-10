#include "ble_config.h"

/*
   This is based on Simple Bluetooth Demo from Liang. His comments are left in here intentionally
   I have added a 2-way communication (also per Liang's example) + lights management

   Created by Liang He, April 27th, 2018

   The Library is created based on Bjorn's code for RedBear BLE communication:
   https://github.com/bjo3rn/idd-examples/tree/master/redbearduo/examples/ble_led

   Our code is created based on the provided example code (Simple Controls) by the RedBear Team:
   https://github.com/RedBearLab/Android
*/

#if defined(ARDUINO)
SYSTEM_MODE(SEMI_AUTOMATIC);
#endif

#define RECEIVE_MAX_LEN 3
#define POT_CHANGE_THRESHOLD 40
#define SEND_MAX_LEN 3
#define BLE_SHORT_NAME_LEN 0x06                // must be in the range of [0x01, 0x09]
#define BLE_SHORT_NAME 'A', 'L', 'I', 'N', 'a' // define each char but the number of char should be BLE_SHORT_NAME_LEN-1

/* Define the pins on the Duo board
   TODO: change the pins here for your applications
*/
#define DIGITAL_OUT_PIN D3

// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16] = {0x71, 0x3d, 0x00, 0x00, 0x50, 0x3e, 0x4c, 0x75, 0xba, 0x94, 0x31, 0x48, 0xf1, 0x8d, 0x94, 0x1e};
static uint8_t service1_tx_uuid[16] = {0x71, 0x3d, 0x00, 0x03, 0x50, 0x3e, 0x4c, 0x75, 0xba, 0x94, 0x31, 0x48, 0xf1, 0x8d, 0x94, 0x1e};
static uint8_t service1_rx_uuid[16] = {0x71, 0x3d, 0x00, 0x02, 0x50, 0x3e, 0x4c, 0x75, 0xba, 0x94, 0x31, 0x48, 0xf1, 0x8d, 0x94, 0x1e};

// Define the configuration data
static uint8_t adv_data[] = {
    0x02,
    BLE_GAP_AD_TYPE_FLAGS,
    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,

    BLE_SHORT_NAME_LEN,
    BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
    BLE_SHORT_NAME,

    0x11,
    BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE,
    0x1e, 0x94, 0x8d, 0xf1, 0x48, 0x31, 0x94, 0xba, 0x75, 0x4c, 0x3e, 0x50, 0x00, 0x00, 0x3d, 0x71};

static btstack_timer_source_t send_characteristic;
// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve
static uint16_t send_handle = 0x0000;    // send

static uint8_t receive_data[RECEIVE_MAX_LEN] = {0x01};
static uint8_t send_data[SEND_MAX_LEN] = {0x00};

/* Pins setup */
int redPin = D2;
int greenPin = D1;
int bluePin = D0;

int potPin = A0;
int photoPin = A1;

// gloabls for read-in values
volatile float g_photoValue;
volatile int g_potValue;

//processed color values form phone
volatile int phoneRed;
volatile int phoneGreen;
volatile int phoneBlue;

//color values from  potentiometer
volatile int potRed;
volatile int potGreen;
volatile int potBlue;

const int DEBOUNCE_DELAY = 200; //in milliseconds
//smoothing
const int SMOOTHING_WINDOW_SIZE = 10;
volatile int _readings[SMOOTHING_WINDOW_SIZE]; // the readings from the analog input
volatile int _readIndex = 0;                   // the index of the current reading
volatile int _total = 0;                       // the running total
volatile int _average = 0;

volatile int _oldReading = 0; // the average - used to check if we are reading phone or potentiometer

/**
   @brief Callback for writing event.

   @param[in]  value_handle
   @param[in]  *buffer       The buffer pointer of writting data.
   @param[in]  size          The length of writting data.

   @retval
*/
int bleWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size)
{
  Serial.print("Write value handler: ");
  Serial.println(value_handle, HEX);

  if (receive_handle == value_handle)
  {
    memcpy(receive_data, buffer, RECEIVE_MAX_LEN);
    Serial.print("Write value: ");
    for (uint8_t index = 0; index < RECEIVE_MAX_LEN; index++)
    {
      Serial.print(receive_data[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");

    if (receive_data[0] == 0x02)
    {
      Serial.println(receive_data[2]);
      Serial.println((int)receive_data[2]);
      Serial.println((int)receive_data[2] & 0xFF);
      int data = receive_data[2] & 0xFF;

      if (receive_data[1] == 0x00) { // red
        phoneRed = data;
      }
      else if (receive_data[1] == 0x01) { // green
        phoneGreen = data;
      }

      else if (receive_data[1] == 0x02) { // blue
        phoneBlue = data;
      }
    }
    else if (receive_data[0] == 0x04)
    { // Command is to initialize all.
        phoneRed = 255;
        phoneGreen = 255;
        phoneBlue = 255;
    }

    else if (receive_data[0] == 0x05)
    {
      Serial.print("shake");
      analogWrite(redPin, 0);
      analogWrite(greenPin, 255);
      analogWrite(bluePin, 255);
    }
  }
  return 0;
}

void printDebug(int number, char message[])
{
  Serial.print(message);
  Serial.println(number);
}

void setup()
{
  Serial.begin(115200);

  delay(5000);
  Serial.println("Simple Digital Out Demo.");

  // Initialize ble_stack.
  ble.init();
  configureBLE(); //lots of standard initialization hidden in here - see ble_config.cpp
  // Set BLE advertising data
  ble.setAdvertisementData(sizeof(adv_data), adv_data);

  // Register BLE callback functions
  ble.onDataWriteCallback(bleWriteCallback);

  // Add user defined service and characteristics
  ble.addService(service1_uuid);
  receive_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY | ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, receive_data, RECEIVE_MAX_LEN);
  send_handle = ble.addCharacteristicDynamic(service1_rx_uuid, ATT_PROPERTY_NOTIFY, send_data, SEND_MAX_LEN);

  // BLE peripheral starts advertising now.
  ble.startAdvertising();
  Serial.println("BLE start advertising.");

  /*
     TODO: This is where you can initialize all peripheral/pin modes
  */
  pinMode(DIGITAL_OUT_PIN, OUTPUT);

  /* alina color setup*/

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(potPin, INPUT);   //set potentiometer for red LED as input
  pinMode(photoPin, INPUT); //set potentiometer for green LED as input

  //smoothing
  for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++)
  {
    _readings[i] = 0;
  }

  // Start a task to check status.
  send_characteristic.process = &send_notify;
  ble.setTimer(&send_characteristic, 500); //2000ms
  ble.addTimer(&send_characteristic);
}

void loop()
{
}

/* Color setting aux functions*/

float getAdjustedPhotoValue(int rawPhotoValue)
{

  float newPhotoValue = map(rawPhotoValue, 1000, 3900, 100, 0);
  newPhotoValue = constrain(newPhotoValue, 0, 100);
  newPhotoValue = newPhotoValue / 100.0;
  return newPhotoValue;
}

/* From Jon's example */
int getSmoothedPotReading(int curReading)
{
  _total = _total - _readings[_readIndex];
  _readings[_readIndex] = curReading;
  _total = _total + _readings[_readIndex];
  _readIndex = _readIndex + 1;
  // if we're at the end of the array...
  if (_readIndex >= SMOOTHING_WINDOW_SIZE)
  {
    _readIndex = 0;
  }

  _average = _total / SMOOTHING_WINDOW_SIZE;
  return _average;
}

/* maps potentiometer values and stores them */
void getColorFromPotentiometer(int potValue)
{

  if (potValue < 1364) // Lowest third of the potentiometer's range (0-340)
  {
    potValue = map(potValue, 0, 1363, 0, 255);
    potRed = 256 - potValue;
    potGreen = potValue;
    potBlue = 1;
  }
  else if (potValue < 2728) // Middle third of potentiometer's range (341-681)
  {
    potValue = map(potValue, 1364, 2727, 0, 255); // Normalize to 0-255
    potRed = 1;
    potGreen = 256 - potValue;
    potBlue = potValue;
  }
  else // Upper third of potentiometer"s range (682-1023)
  {
    potValue = map(potValue, 2728, 4092, 0, 255); // Normalize to 0-255
    potRed  = potValue;
    potGreen = 1;
    potBlue = 256 - potValue;;
  }
  phoneRed = potRed;
  phoneGreen = potGreen;
  phoneBlue = potBlue;
}

void setColor(int r, int g, int b, bool isSend)
{
  setColorIndividual(redPin, r, isSend);
  setColorIndividual(greenPin, g, isSend);
  setColorIndividual(bluePin, b, isSend);
}

void setColorIndividual(int pin, int color, bool isSend)
{
  int colorAdjusted = (int)color * g_photoValue;
  int writeValue_color = 255 - colorAdjusted;
  if (isSend){
    send(pin, color);
  }
  analogWrite(pin, writeValue_color);
}

/* send data back to android */
void send(int pin, int c)
{

  byte pinByte = 0x00;
  if (pin == greenPin)
    pinByte = 0x01;
  else if (pin == bluePin)
    pinByte = 0x02;
  send_data[0] = (pinByte);
  send_data[1] = byte(c);
  send_data[2] = 0x03;

  if (ble.attServerCanSendPacket())
    ble.sendNotify(send_handle, send_data, SEND_MAX_LEN);
}

//this is our loop
static void send_notify(btstack_timer_source_t *ts)
{
  g_photoValue = getAdjustedPhotoValue(analogRead(photoPin));
  g_potValue = getSmoothedPotReading(analogRead(potPin));
  if (abs(_oldReading - g_potValue) > POT_CHANGE_THRESHOLD)
  {
    _oldReading = g_potValue;
    printDebug(g_potValue, "pot value change");
    getColorFromPotentiometer(g_potValue);
    setColor(potRed, potGreen, potBlue, true);
  }
  else
  {
    setColor(phoneRed, phoneGreen, phoneBlue, false);
  }

  ble.setTimer(ts, 200);
  ble.addTimer(ts);
}
