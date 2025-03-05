/*!
  * @file  mRangeVelocity.ino
  * @brief  radar measurement demo
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V1.0
  * @date 2024-02-02
  * @url https://github.com/dfrobot/DFRobot_C4001
  */


// Radar code
// Edited from DFRobot example code for theSEN0609_C4001_mmWave_Presence_Sensor_25m
// runs on an Arduino Uno
// Artist / Creative Director: Elgin Rust
// Technical Direction and Arduino code by Duncan Greenwood, possibly with help from Rikus Wessels


#include "DFRobot_C4001.h"

//#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef I2C_COMMUNICATION
/*
   * DEVICE_ADDR_0 = 0x2A     default iic_address
   * DEVICE_ADDR_1 = 0x2B
   */
DFRobot_C4001_I2C radar(&Wire, DEVICE_ADDR_0);
#else
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed */
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)

SoftwareSerial mySerial1(4, 5);
DFRobot_C4001_UART radar1(&mySerial1, 9600);


SoftwareSerial mySerial2(8, 9);
DFRobot_C4001_UART radar2(&mySerial2, 9600);

//SoftwareSerial mySerialBLE(2, 3);


#elif defined(ESP32)
DFRobot_C4001_UART radar(&Serial1, 9600, /*rx*/ D2, /*tx*/ D3);
#else
DFRobot_C4001_UART radar(&Serial1, 9600);
#endif
#endif

int SensorSelect = 1;   // Start reading from the 1st radar
int TD = 2000;          // sensor time delay
int logTD = 100;          // logs time delay


// define pins for digital write to BLE transmitter
int radar1pin = 10;
int radar2pin = 11;


void setup() {
  pinMode(radar1pin, OUTPUT);
  pinMode(radar2pin, OUTPUT);


  Serial.begin(115200);

  //while (!Serial);
  
  while (!radar1.begin()) {
    Serial.println("NO Deivces !");
    delay(logTD);
  }
  Serial.println("Radar 1 connected!");
  
//  settings for radar1

  // speed Mode
  radar1.setSensorMode(eSpeedMode);

  sSensorStatus_t data1;
  data1 = radar1.getStatus();
  //  0 stop  1 start
  Serial.print("radar1 work status  = ");
  Serial.println(data1.workStatus);

  //  0 is exist   1 speed
  Serial.print("radar1 work mode  = ");
  Serial.println(data1.workMode);

  //  0 no init    1 init success
  Serial.print("radar1 init status = ");
  Serial.println(data1.initStatus);
  Serial.println();

  //Serial.print("Opening connection to Bluetooth transmitter");
  //mySerialBLE.begin(9600);

  /*
   * min Detection range Minimum distance, unit cm, range 0.3~20m (30~2500), not exceeding max, otherwise the function is abnormal.
   * max Detection range Maximum distance, unit cm, range 2.4~20m (240~2500)
   * thres Target detection threshold, dimensionless unit 0.1, range 0~6553.5 (0~65535)
   */

  
  if (radar1.setDetectThres(/*min*/ 40, /*max*/ 1800, /*thres*/ 10)) {
    Serial.println("radar1 set detect threshold successfully");
  }
  

  // set Fretting Detection
  radar1.setFrettingDetection(eON);

  // get confige params
  /*
  Serial.print("radar1 min range = ");
  Serial.println(radar1.getTMinRange());
  Serial.print("radar1 max range = ");
  Serial.println(radar1.getTMaxRange());
  Serial.print("radar1 threshold range = ");
  Serial.println(radar1.getThresRange());
  Serial.print("radar1 fretting detection = ");
  Serial.println(radar1.getFrettingDetection());
*/


//  settings for radar2
  while (!radar2.begin()) {
    Serial.println("NO Deivces !");
    delay(logTD);
  }
  Serial.println("Radar 2 connected!");

  // speed Mode
  radar2.setSensorMode(eSpeedMode);

  sSensorStatus_t data2;
  data2 = radar2.getStatus();
  //  0 stop  1 start
  Serial.print("radar2 work status  = ");
  Serial.println(data2.workStatus);

  //  0 is exist   1 speed
  Serial.print("radar2 work mode  = ");
  Serial.println(data2.workMode);

  //  0 no init    1 init success
  Serial.print("radar2 init status = ");
  Serial.println(data2.initStatus);
  Serial.println();

  /*
   * min Detection range Minimum distance, unit cm, range 0.3~20m (30~2500), not exceeding max, otherwise the function is abnormal.
   * max Detection range Maximum distance, unit cm, range 2.4~20m (240~2500)
   * thres Target detection threshold, dimensionless unit 0.1, range 0~6553.5 (0~65535)
   */

  
  if (radar2.setDetectThres(/*min*/ 40, /*max*/ 1800, /*thres*/ 10)) {
    Serial.println("radar2 set detect threshold successfully");
  }
  

  // set Fretting Detection
  radar2.setFrettingDetection(eON);

  // get confige params
  /*
  Serial.print("radar2 min range = ");
  Serial.println(radar2.getTMinRange());
  Serial.print("radar2 max range = ");
  Serial.println(radar2.getTMaxRange());
  Serial.print("radar2 threshold range = ");
  Serial.println(radar2.getThresRange());
  Serial.print("radar2 fretting detection = ");
  Serial.println(radar2.getFrettingDetection());
  */
}

void loop() {
  switch (SensorSelect)
  {
    
    case 1: // read from radar 1
      Serial.println("Attempting to read sensor 1...");
      mySerial1.begin(9600);

      Serial.println("Reading sensor 1...");

      Serial.print("radar1 target number = ");
      Serial.println(radar1.getTargetNumber());  // must exist
      Serial.print("radar1 target Speed  = ");
      Serial.print(radar1.getTargetSpeed());
      Serial.println(" m/s");

      Serial.print("radar1 target range  = ");
      Serial.print(radar1.getTargetRange());
      Serial.println(" m");

      Serial.print("radar1 target energy  = ");
      Serial.println(radar1.getTargetEnergy());
      Serial.println();

/*
      if (mySerialBLE.available())
      {
        if (radar1.getTargetEnergy() > 300000)
          {
            Serial.println("Radar 1 Energy high)");
            mySerialBLE.print("11");
          }
        else
          mySerialBLE.print("10");
      }
      */

      if (radar1.getTargetNumber() == 1){
        if (radar1.getTargetEnergy() > 10000)
            {
              Serial.println("Radar 1 Energy high)");
              digitalWrite(radar1pin, HIGH);  // pin 10 write 
            }
        else
          {
              Serial.println("Radar 1 Energy low)");
              digitalWrite(radar1pin, LOW);  //
          }
      }
      else {
        Serial.println("Nobody on Radar 1)");
        digitalWrite(radar1pin, LOW);  //
      }

      
      delay(TD);
      SensorSelect = 2;  // Read the next sensor

  break;

    case 2: // read from radar 2

      Serial.println("Attempting to read sensor 2...");
      mySerial2.begin(9600);

      Serial.println("Reading sensor 2...");
      Serial.print("radar2 target number = ");
      Serial.println(radar2.getTargetNumber());  // must exist
      Serial.print("radar2target Speed  = ");
      Serial.print(radar2.getTargetSpeed());
      Serial.println(" m/s");

      Serial.print("radar2 target range  = ");
      Serial.print(radar2.getTargetRange());
      Serial.println(" m");

      Serial.print("radar2 target energy  = ");
      Serial.println(radar2.getTargetEnergy());
      Serial.println();
/*
      if (mySerialBLE.available())
      {
        if (radar2.getTargetEnergy() > 300000)
          {
            Serial.println("Radar 2 Energy high)");
          mySerialBLE.print("21");
          }
        else
          mySerialBLE.print("20");
      }
      */

      if (radar2.getTargetNumber() == 1){
        if (radar2.getTargetEnergy() > 10000)
            {
              Serial.println("Radar 2 Energy high)");
              digitalWrite(radar2pin, HIGH);  // pin 10 write 
            }
        else
          {
              Serial.println("Radar 2 Energy low)");
              digitalWrite(radar2pin, LOW);  //
          }
      }
      else {
        Serial.println("Nobody on Radar 2)");
        digitalWrite(radar2pin, LOW);  //
      }

      delay(TD);
    
      SensorSelect = 1;  // Read the 1st radar

      break;
  }
}