// receiver code
// runs on a Seeed Xiao NRF52840 Sense
// Artist / Creative Director: Elgin Rust
// Technical Direction and Arduino code by Duncan Greenwood, possibly with help from Rikus Wessels


#include <ArduinoBLE.h>

#include <Adafruit_NeoPixel.h>
#define PIN 1
#define NUMPIXELS 150
#define DELAY 2

#define NUM_LEDS 150  // Number of LEDs per strip
#define NUM_STRIPS 4  // Total strips
#define BRIGHTNESS 64  // Max brightness

// Define the pin numbers for the LED strips
#define LED_PIN_1 0  // Adjust as needed
#define LED_PIN_2 1
#define LED_PIN_3 2
#define LED_PIN_4 3

// Color definitions taken from Elgin's swatches picked out of Photoshop
#define PINK 169, 39, 148  // #a92794
#define GREEN 32, 219, 116  // 
#define YELLOW 215, 198, 94
#define PURPLE 107, 59, 151

// Create NeoPixel objects for each strip
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(NUM_LEDS, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_LEDS, LED_PIN_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(NUM_LEDS, LED_PIN_3, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip4 = Adafruit_NeoPixel(NUM_LEDS, LED_PIN_4, NEO_GRB + NEO_KHZ800);


BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = LED_BUILTIN; // pin to use for the LED

void setup() {
  Serial.begin(9600);
  //while (!Serial);

  strip1.begin();
  strip2.begin();
  strip3.begin();
  strip4.begin();

// I'm not sure if this is reduntant, as the brightness is also set in the 
  strip1.setBrightness(BRIGHTNESS);
  strip2.setBrightness(BRIGHTNESS);
  strip3.setBrightness(BRIGHTNESS);
  strip4.setBrightness(BRIGHTNESS);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("XIAO");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  // print address
  Serial.print("Address: ");
  Serial.println(BLE.address());

  Serial.println("XIAO nRF52840 Peripheral");
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value() == (byte)0x03) {   // both radar signals are HIGH
          Serial.println("I see radar 1 AND 2");
         // make wgite
          digitalWrite(LED_GREEN, LOW);  // turn the Green LED on 
          digitalWrite(LED_BLUE, LOW);  // turn the Blue LED on 
          digitalWrite(LED_RED, LOW);  // turn the Red LED on 

          // fill strip with PURPLE (test)
          fillStrip(strip2, PURPLE, BRIGHTNESS);

        } else if (switchCharacteristic.value() == (byte)0x01) {   // Radar1 HIGH Radar2 LOW
          Serial.println("I see Radar 1, but not 2");
                  // make green
          digitalWrite(LED_GREEN, LOW);  // turn the Green LED on 
          digitalWrite(LED_BLUE, HIGH);  // turn the Blue LED off
          digitalWrite(LED_RED, HIGH);  // turn the Red LED off

          // fill strip with PINK (test)
          fillStrip(strip2, PINK, BRIGHTNESS);


        }
        else if (switchCharacteristic.value() == (byte)0x02) {   // Radar1 LOW Radar2 HIGH
          Serial.println("I see only Radar 2.");
          // make blue
          digitalWrite(LED_GREEN, HIGH);  // turn the Green LED off 
          digitalWrite(LED_BLUE, LOW);  // turn the Blue LED on
          digitalWrite(LED_RED, HIGH);  // turn the Red LED off 
          // fill strip with YELLOW (test)
          fillStrip(strip2, YELLOW, BRIGHTNESS);


        }
        else if (switchCharacteristic.value() == (byte)0x00) {   // Both Radars LOW
          Serial.println("Nothing to see here.");
          // make red
          digitalWrite(LED_GREEN, HIGH);  // turn the Green LED off 
          digitalWrite(LED_BLUE, HIGH);  // turn the Blue LED off
          digitalWrite(LED_RED, LOW);  // turn the Red LED on 

          // fill strip with GREEN (test)
          fillStrip(strip2, GREEN, BRIGHTNESS);

        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

// Helper function to fill an LED strip with a color at a specific brightness
void fillStrip(Adafruit_NeoPixel &strip, int r, int g, int b, int brightness) {
    strip.setBrightness(brightness);
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}