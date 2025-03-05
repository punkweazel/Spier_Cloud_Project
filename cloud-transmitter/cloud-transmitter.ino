// transmitter code
// runs on a Seeed Xiao NRF52840 Sense


#include <ArduinoBLE.h>
//#include U8x8lib.h
#include <Wire.h>

int radar1pin = 4;
int radar2pin = 5;

int oldRadar1Val = LOW;
int oldRadar2Val = LOW;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(radar1pin, INPUT);
  pinMode(radar2pin, INPUT);

  Serial.begin(9600);
  //while (!Serial);

 //  initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central - LED control");

//   start scanning for peripherals
  BLE.scanForName("XIAO");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "XIAO") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    system_control(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForName("XIAO");
  }
  delay(100);
}

void system_control(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    // while the peripheral is connected
    // read the button pin
      int radar1val = digitalRead(radar1pin);
      int radar2val = digitalRead(radar2pin);

    if ((oldRadar1Val != radar1val) || (oldRadar2Val != radar2val)) {
      // at least one of the radars changed
      oldRadar1Val = radar1val;
      oldRadar2Val = radar2val;


    if ((radar1val) && (radar2val)) {
        Serial.println("Radar 1 AND 2 HIGH");

        // both radars are HIGH, write 0x03 to turn the LED on
        ledCharacteristic.writeValue((byte)0x03);

        // make wgite
        digitalWrite(LED_GREEN, LOW);  // turn the Green LED on 
        digitalWrite(LED_BLUE, LOW);  // turn the Blue LED on 
        digitalWrite(LED_RED, LOW);  // turn the Red LED on 


      } else if (radar1val){
        Serial.println("Radar1 HIGH Radar2 LOW");

        // Radar1 HIGH Radar2 LOW, write 0x01 
        ledCharacteristic.writeValue((byte)0x01);

        // make green
        digitalWrite(LED_GREEN, LOW);  // turn the Green LED on 
        digitalWrite(LED_BLUE, HIGH);  // turn the Blue LED off
        digitalWrite(LED_RED, HIGH);  // turn the Red LED off 
      }
      else if (radar2val){
        Serial.println("Radar1 LOW Radar2 HIGH");

        // Radar1 LOW Radar2 HIGH, write 0x02 
        ledCharacteristic.writeValue((byte)0x02);
        // make blue
        digitalWrite(LED_GREEN, HIGH);  // turn the Green LED off 
        digitalWrite(LED_BLUE, LOW);  // turn the Blue LED on
        digitalWrite(LED_RED, HIGH);  // turn the Red LED off 
      }
      else {
        Serial.println("Both Radars LOW");
        ledCharacteristic.writeValue((byte)0x00);
        // make red
        digitalWrite(LED_GREEN, HIGH);  // turn the Green LED off 
        digitalWrite(LED_BLUE, HIGH);  // turn the Blue LED off
        digitalWrite(LED_RED, LOW);  // turn the Red LED on 
      }
    }
  }

  Serial.println("Peripheral disconnected");
}




/////


