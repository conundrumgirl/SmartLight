/*
   Changing colors with epotentiometer */
#if defined(ARDUINO) 
SYSTEM_MODE(SEMI_AUTOMATIC); 
#endif

/*
  Adafruit Arduino - Lesson 3. RGB LED
*/

int redPin = D2;
int greenPin = D1;
int bluePin = D0;

int rx;
int gx;
int bx;


int potPin = A0;
int photoPin = A1;



int potValue;
int photoValue;
float newPhotoValue;



int writeValue_red; //declare variable to send to the red LED
int writeValue_green; //declare variable to send to the green LED
int writeValue_blue; //declare variable to send to the blue LED

/************ bluetooth setup
 *  
 */

#define RECEIVE_MAX_LEN    3
#define SEND_MAX_LEN    3
#define BLE_SHORT_NAME_LEN 0x06 // must be in the range of [0x01, 0x09]
#define BLE_SHORT_NAME 'A','L','I','N','A'  // define each char but the number of char should be BLE_SHORT_NAME_LEN-1

/* Define the pins on the Duo board
 * TODO: change the pins here for your applications
 */
#define DIGITAL_OUT_PIN            D3
//smoothing code from Jon
// Define the smoothing window size
const int SMOOTHING_WINDOW_SIZE = 10;


int _readings[SMOOTHING_WINDOW_SIZE];      // the readings from the analog input
int _readIndex = 0;              // the index of the current reading
int _total = 0;                  // the running total
int _average = 0;                // the average

//bluetooth stuff
// UUID is used to find the device by other BLE-abled devices
static uint8_t service1_uuid[16]    = { 0x71,0x3d,0x00,0x00,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };
static uint8_t service1_tx_uuid[16] = { 0x71,0x3d,0x00,0x03,0x50,0x3e,0x4c,0x75,0xba,0x94,0x31,0x48,0xf1,0x8d,0x94,0x1e };

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
  0x1e,0x94,0x8d,0xf1,0x48,0x31,0x94,0xba,0x75,0x4c,0x3e,0x50,0x00,0x00,0x3d,0x71 
};

// Define the receive and send handlers
static uint16_t receive_handle = 0x0000; // recieve

static uint8_t receive_data[RECEIVE_MAX_LEN] = { 0x01 };

/**
 * @brief Callback for writing event.
 *
 * @param[in]  value_handle  
 * @param[in]  *buffer       The buffer pointer of writting data.
 * @param[in]  size          The length of writting data.   
 *
 * @retval 
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
     * TODO: Receive the data sent from other BLE-abled devices (e.g., Android app)
     * and process the data for different purposes (digital write, digital read, analog read, PWM write)
     */
    if (receive_data[0] == 0x01) { // Command is to control digital out pin
      if (receive_data[1] == 0x01)
        digitalWrite(DIGITAL_OUT_PIN, HIGH);
      else
        digitalWrite(DIGITAL_OUT_PIN, LOW);
    }
    else if (receive_data[0] == 0x04) { // Command is to initialize all.
      digitalWrite(DIGITAL_OUT_PIN, LOW);
    }
  }
  return 0;
}
//end of bluetooth stuff

void setup()
{
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(potPin, INPUT); //set potentiometer for red LED as input
  pinMode(photoPin, INPUT); //set potentiometer for green LED as input

  Serial.begin(9600);

 // initialize all the readings to 0
  for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
    _readings[i] = 0;
  }

//bluetooth stuff

// Initialize ble_stack.
  ble.init();
  configureBLE(); //lots of standard initialization hidden in here - see ble_config.cpp
  // Set BLE advertising data
  ble.setAdvertisementData(sizeof(adv_data), adv_data);

  // Register BLE callback functions
  ble.onDataWriteCallback(bleWriteCallback);

  // Add user defined service and characteristics
  ble.addService(service1_uuid);
  receive_handle = ble.addCharacteristicDynamic(service1_tx_uuid, ATT_PROPERTY_NOTIFY|ATT_PROPERTY_WRITE|ATT_PROPERTY_WRITE_WITHOUT_RESPONSE, receive_data, RECEIVE_MAX_LEN);
  
  // BLE peripheral starts advertising now.
  ble.startAdvertising();
  Serial.println("BLE start advertising.");

 /*
   * TODO: This is where you can initialize all peripheral/pin modes
   */
  pinMode(DIGITAL_OUT_PIN, OUTPUT);

  
  setColor(255, 0, 0 , 0);


}

void loop()
{
  potValue = analogRead(potPin);
  photoValue = analogRead(photoPin);

  Serial.print("Raw photocell value:");
  Serial.println(photoValue);
 
  getColorFromPotentiometer(potValue, photoValue);

  Serial.print("Original photo value");
  Serial.println(photoValue);



  delay(200);
}

int smooth(int curReading) {
    // subtract the last reading
  _total = _total - _readings[_readIndex];
  
  // read from the sensor
  //int curReading = analogRead(ANALOG_INPUT_PIN);
  _readings[_readIndex] = curReading;
  
  // add the reading to the total
  _total = _total + _readings[_readIndex];
  
  // advance to the next position in the array
  _readIndex = _readIndex + 1;

  // if we're at the end of the array...
  if (_readIndex >= SMOOTHING_WINDOW_SIZE) {
    // ...wrap around to the beginning:
    _readIndex = 0;
  }

  // calculate the average
  _average = _total / SMOOTHING_WINDOW_SIZE;
  
  // send it to the computer as ASCII digits
  Serial.print(curReading);
  Serial.print(",");
  Serial.println(_average);
  return _average;
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
  newPhotoValue = (map(photoValue, 1000, 3900, 100, 0)) / 100.0;
  if (newPhotoValue < 0) {
    newPhotoValue = 0;
  }
  if (newPhotoValue > 100) {
    newPhotoValue = 100;
  }

  Serial.print("New photo value");
  Serial.println(newPhotoValue);

  rx = r * newPhotoValue;
  gx = g * newPhotoValue;
  bx = b * newPhotoValue;
  Serial.println(rx);
  writeValue_red = 255 - rx ;       // Red from off to full
  writeValue_green = 255 - gx;            // Green off
  writeValue_blue = 255 - bx;

  Serial.print("writeValue_red" +  writeValue_red);
  Serial.print("newPhoto");
  Serial.println(newPhotoValue);
  analogWrite(redPin, writeValue_red);
  analogWrite(greenPin, writeValue_green);
  analogWrite(bluePin, writeValue_blue);

  // Serial.print(writeValue_red);
  // Serial.print(",");
  // Serial.print(writeValue_blue);
  // Serial.print(",");
  // Serial.println(writeValue_green);



}





