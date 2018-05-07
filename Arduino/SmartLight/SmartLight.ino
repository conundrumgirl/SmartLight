#include "ble_config.h"

/*
   Simple Bluetooth Demo
   This code shows that the user can send simple digital write data from the
   Android app to the Duo board.
   Created by Liang He, April 27th, 2018

   The Library is created based on Bjorn's code for RedBear BLE communication:
   https://github.com/bjo3rn/idd-examples/tree/master/redbearduo/examples/ble_led

   Our code is created based on the provided example code (Simple Controls) by the RedBear Team:
   https://github.com/RedBearLab/Android
*/

#if defined(ARDUINO)
SYSTEM_MODE(SEMI_AUTOMATIC);
#endif

#define RECEIVE_MAX_LEN    3
#define BLE_SHORT_NAME_LEN 0x06                                                                                                                                                                                                                                                                                               // must be in the range of [0x01, 0x09]
#define BLE_SHORT_NAME 'A','L','I','N', 'a'  // define each char but the number of char should be BLE_SHORT_NAME_LEN-1

/* Define the pins on the Duo board
   TODO: change the pins here for your applications
*/
#define DIGITAL_OUT_PIN            D3


// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16]    = { 0x71, 0x3d, 0x00, 0x00, 0x50, 0x3e, 0x4c, 0x75, 0xba, 0x94, 0x31, 0x48, 0xf1, 0x8d, 0x94, 0x1e };
static uint8_t service1_tx_uuid[16] = { 0x71, 0x3d, 0x00, 0x03, 0x50, 0x3e, 0x4c, 0x75, 0xba, 0x94, 0x31, 0x48, 0xf1, 0x8d, 0x94, 0x1e };

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
  0x1e, 0x94, 0x8d, 0xf1, 0x48, 0x31, 0x94, 0xba, 0x75, 0x4c, 0x3e, 0x50, 0x00, 0x00, 0x3d, 0x71
};

// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve

static uint8_t receive_data[RECEIVE_MAX_LEN] = { 0x01 };


/* alina color setup

*/

int redPin = D2;
int greenPin = D1;
int bluePin = D0;

//int rx;
//int gx;
//int bx;


int potPin = A0;
int photoPin = A1;



//int potValue;
int photoValue;
//float newPhotoValue;



//int writeValue_red; //declare variable to send to the red LED
//int writeValue_green; //declare variable to send to the green LED
//int writeValue_blue; //declare variable to send to the blue LED


/**
   @brief Callback for writing event.

   @param[in]  value_handle
   @param[in]  *buffer       The buffer pointer of writting data.
   @param[in]  size          The length of writting data.

   @retval
*/
int bleWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  Serial.print("Write value handler: ");
  Serial.println(value_handle, HEX);

  if (receive_handle == value_handle) {
    memcpy(receive_data, buffer, RECEIVE_MAX_LEN);
    Serial.print("Write value: ");
    for (uint8_t index = 0; index < RECEIVE_MAX_LEN; index++) {
      Serial.print(receive_data[index], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");





    /* Process the data
       TODO: Receive the data sent from other BLE-abled devices (e.g., Android app)
       and process the data for different purposes (digital write, digital read, analog read, PWM write)
    */
    //digital
    /*if (receive_data[0] == 0x01) { // Command is to control digital out pin
      if (receive_data[1] == 0x01)
        digitalWrite(DIGITAL_OUT_PIN, HIGH);
      else
        digitalWrite(DIGITAL_OUT_PIN, LOW);
    }
    else if (receive_data[0] == 0x04) { // Command is to initialize all.
      digitalWrite(DIGITAL_OUT_PIN, LOW);
    }*/

   /* Process the data
     * TODO: Receive the data sent from other BLE-abled devices (e.g., Android app)
     * and process the data for different purposes (digital write, digital read, analog read, PWM write)
     */
    if(receive_data[0] == 0x02) { // Command is to control PWM pin
      int data = 255-(receive_data[2]& 0xFF);

      if(receive_data[1] == 0x00) { // Command is to control PWM pin
        printDebug(data, "red value");
        analogWrite(redPin, data);

      }
      if(receive_data[1] == 0x01) { // Command is to control PWM pin
        printDebug(data, "green value");
        analogWrite(greenPin, data);
      }

      if(receive_data[1] == 0x02) { // Command is to control PWM pin
         printDebug(data, "blue value");
        analogWrite(bluePin, data);
      }




  //Serial.println(newPhotoValue);
     // analogWrite(redPin, 255-(receive_data[1]& 0xFF));
      //analogWrite(greenPin, 255-(receive_data[2]& 0xFF));
      //analogWrite(bluePin, 255-(receive_data[3]& 0xFF));

    }
    else if (receive_data[0] == 0x04) { // Command is to initialize all.
      analogWrite(redPin, 255);
      analogWrite(greenPin, 255);
      analogWrite(bluePin, 255);
    }




  }
  return 0;
}

void printDebug(int number, char message[]) {
Serial.print(message);
//strcpy(message, "Hello, world!")
//printf("%s\n", message);
Serial.println(number);
}

void setup() {
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

  pinMode(potPin, INPUT); //set potentiometer for red LED as input
  pinMode(photoPin, INPUT); //set potentiometer for green LED as input


}

void loop() {

  //photoValue = getAdjustedPhotoValue(analogRead(photoPin));
  //float newPhotoValue = (map(photoValue, 1000, 3900, 100, 0)) / 100.0;

  //photoValue = constrain(newPhotoValue, 0, 255);
  //doColorLoop(potPin, photoPin);
 // delay(200);
}

/* Color setting aux functions*/

int getAdjustedPhotoValue(int rawPhotoValue) {
   float newPhotoValue = (map(rawPhotoValue, 1000, 3900, 100, 0)) / 100.0;

  newPhotoValue = constrain(newPhotoValue, 0, 255);
  return newPhotoValue;
}

void doColorLoop(int _potPin, int photoValue) {
  int potValue = analogRead(_potPin);


  //Serial.print("Raw photocell value:");
  //Serial.println(photoValue);

  getColorFromPotentiometer(potValue, photoValue);

  //Serial.print("Original photo value");
  //Serial.println(photoValue);

}

void getColorFromPotentiometer(int potValue, int photoValue) {
  if (potValue < 1364)  // Lowest third of the potentiometer's range (0-340)
  {
    potValue =  map(potValue, 0, 1363, 0, 255);
    setColor(256 - potValue,  potValue, 1, photoValue);
  }
  else if (potValue < 2728) // Middle third of potentiometer's range (341-681)
  {
    potValue =  map(potValue, 1364, 2727, 0, 255); // Normalize to 0-255
    setColor(1, 256 - potValue, potValue, photoValue);

  }
  else  // Upper third of potentiometer"s range (682-1023)
  {
    potValue =  map(potValue, 2728, 4092, 0, 255); // Normalize to 0-255
    setColor(potValue, 1, 256 - potValue, photoValue);
  }
}

void setColor(int r, int g, int b, int photoValue) {
 // float newPhotoValue = (map(photoValue, 1000, 3900, 100, 0)) / 100.0;

  //newPhotoValue = constrain(newPhotoValue, 0, 255);

  //Serial.print("New photo value");
  //Serial.println(newPhotoValue);

  /*int rx = r * photoValue;
  int gx = g * photoValue;
  int bx = b * photoValue;

  int writeValue_red = 255 - rx ;       // Red from off to full
  int writeValue_green = 255 - gx;            // Green off
  int writeValue_blue = 255 - bx;

  analogWrite(redPin, writeValue_red);
  analogWrite(greenPin, writeValue_green);
  analogWrite(bluePin, writeValue_blue);*/
  //setColorIndividual(int redPin, int r, int photoValue );
  // setColorIndividual(int greenPin, int g, int photoValue );
  //  setColorIndividual(int redPin, int b, int photoValue );
}

void setColorIndividual(int pin, int color, int photoValue ) {
  int colorAdjusted = color * photoValue;
  int writeValue_color = 255 - colorAdjusted ;
  analogWrite(pin, writeValue_color);
}
