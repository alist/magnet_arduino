#include <SoftwareSerial.h>
#include "BGLib.h"
#include <Adafruit_NeoPixel.h>
#include <SPI.h>


#define PIN 12
#define PIXELS 6
#define CS 17

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELS, PIN, NEO_GRB + NEO_KHZ800);


// Uncomment to see Debugging stuff in the Serial
// window. NOTE: When running the FemtoduinoBLE without 
// a Serial connection to your computer, do not wait for the 
// Serial object to become available, as (obviously) it won't.
// (Not connected to a USB port, so it's obviously not there, right?)

#define DEBUG

// ================================================================
// BLE STATE TRACKING (UNIVERSAL TO JUST ABOUT ANY BLE PROJECT)
// ================================================================

// BLE state machine definitions
#define BLE_STATE_STANDBY           0
#define BLE_STATE_SCANNING          1
#define BLE_STATE_ADVERTISING       2
#define BLE_STATE_CONNECTING        3
#define BLE_STATE_CONNECTED_MASTER  4
#define BLE_STATE_CONNECTED_SLAVE   5

// BLE state/link status tracker
uint8_t ble_state = BLE_STATE_STANDBY;
uint8_t ble_encrypted = 0;  // 0 = not encrypted, otherwise = encrypted
uint8_t ble_bonding = 0xFF; // 0xFF = no bonding, otherwise = bonding handle

// Pins
#define LED_PIN 15
#define BLE_WAKEUP_PIN 5
#define BLE_RESET_PIN 6

#define RGB_LED_PIN1 9
#define RGB_LED_PIN2 10
#define RGB_LED_PIN3 11
//#define RGB_LED_PIN4 12

#define GATT_HANDLE_C_RX_DATA 17
#define GATT_HANDLE_C_TX_DATA 20


// Let's talk to our BLE module using these ports as RX/TX
SoftwareSerial bleSerialPort(8, 3); // D8 - RX, D3 - TX

BGLib ble113((HardwareSerial *) &bleSerialPort, 0, 1);
#define BGAPI_GET_RESPONSE(v, dType) dType *v = (dType *)ble113.getLastRXPayload()


//accel
//This buffer will hold values read from the ADXL345 registers.
char values[10];
char output[20];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
double xg, yg, zg;
char tapType=0;

//ADXL345 Register Addresses
#define DEVID   0x00  //Device ID Register
#define THRESH_TAP  0x1D  //Tap Threshold
#define OFSX    0x1E  //X-axis offset
#define OFSY    0x1F  //Y-axis offset
#define OFSZ    0x20  //Z-axis offset
#define DURATION  0x21  //Tap Duration
#define LATENT    0x22  //Tap latency
#define WINDOW    0x23  //Tap window
#define THRESH_ACT  0x24  //Activity Threshold
#define THRESH_INACT  0x25  //Inactivity Threshold
#define TIME_INACT  0x26  //Inactivity Time
#define ACT_INACT_CTL 0x27  //Axis enable control for activity and inactivity detection
#define THRESH_FF 0x28  //free-fall threshold
#define TIME_FF   0x29  //Free-Fall Time
#define TAP_AXES  0x2A  //Axis control for tap/double tap
#define ACT_TAP_STATUS  0x2B  //Source of tap/double tap
#define BW_RATE   0x2C  //Data rate and power mode control
#define POWER_CTL 0x2D  //Power Control Register
#define INT_ENABLE  0x2E  //Interrupt Enable Control
#define INT_MAP   0x2F  //Interrupt Mapping Control
#define INT_SOURCE  0x30  //Source of interrupts
#define DATA_FORMAT 0x31  //Data format control
#define DATAX0    0x32  //X-Axis Data 0
#define DATAX1    0x33  //X-Axis Data 1
#define DATAY0    0x34  //Y-Axis Data 0
#define DATAY1    0x35  //Y-Axis Data 1
#define DATAZ0    0x36  //Z-Axis Data 0
#define DATAZ1    0x37  //Z-Axis Data 1
#define FIFO_CTL  0x38  //FIFO control
#define FIFO_STATUS 0x39  //FIFO status



/**
 * Based on Jeff Rowberg's BGLib_stub_slave example. Modified to work 
 * with the FemtoduinoBLE, revision 7 or higher.
 */
void setup() {
  // Let's setup our status LED.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // We will need to set up the BLE control pins.
  // (BLE Reset, BLE wake-up)
  pinMode(BLE_RESET_PIN, OUTPUT);
  digitalWrite(BLE_RESET_PIN, HIGH);
  
  pinMode(BLE_WAKEUP_PIN, OUTPUT);
  digitalWrite(BLE_WAKEUP_PIN, LOW);
  
  pinMode(RGB_LED_PIN1, OUTPUT);
  pinMode(RGB_LED_PIN2, OUTPUT);
  pinMode(RGB_LED_PIN3, OUTPUT);
  //pinMode(RGB_LED_PIN4, OUTPUT);
  
  digitalWrite(RGB_LED_PIN1, LOW);
  digitalWrite(RGB_LED_PIN2, LOW);
  digitalWrite(RGB_LED_PIN3, LOW);
  //digitalWrite(RGB_LED_PIN4, LOW);
  
  // Great! Let's set up our internal status handlers, so we 
  // can blink our status LED accordingly!
  ble113.onBusy = onBusy;
  ble113.onIdle = onIdle;
  ble113.onTimeout = onTimeout;
  
  ble113.onBeforeTXCommand = onBeforeTXCommand;
  ble113.onTXCommandComplete = onTXCommandComplete;
  
  ble113.ble_evt_system_boot = femtoSystemBoot;
  ble113.ble_evt_connection_status = femtoConnectionStatus;
  ble113.ble_evt_connection_disconnected = femtoConnectionDisconnect;
  ble113.ble_evt_attributes_value = femtoAttributesValue;


#ifdef DEBUG
  Serial.begin(38400);
  while(!Serial);
#endif
  // open BLE software serial port
  bleSerialPort.begin(38400);
  
  // reset module (maybe not necessary for your application)
  digitalWrite(BLE_RESET_PIN, LOW);
  delay(5); // wait 5ms
  digitalWrite(BLE_RESET_PIN, HIGH);

  strip.begin();
  // strip.show(); // Initialize all pixels to 'off'



//Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);

  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);

  //Send the Tap and Double Tap Interrupts to INT1 pin //I beleive this disable doubles
  writeRegister(INT_MAP, 0x9F);
  //Look for taps on the Z axis only.
  writeRegister(TAP_AXES, 0x01);
  //Set the Tap Threshold to 3g
  writeRegister(THRESH_TAP, 0x38);
  //Set the Tap Duration that must be reached
  writeRegister(DURATION, 0x10);
  
  //100ms Latency before the second tap can occur.
  writeRegister(LATENT, 0x00); //0x50 was
  writeRegister(WINDOW, 0x00); //0xFF
  
  //Enable the Single and NOT Double Taps., 0b11000000 in hex, 0b11100000 for double
  writeRegister(INT_ENABLE, 0xC0);  
  
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode
  readRegister(INT_SOURCE, 1, values); //Clear the interrupts from the INT_SOURCE register.

  //Create an interrupt that will trigger when a tap is detected.
  // debugLog("pre");
  attachInterrupt(1, tap, CHANGE);
  // debugLog("post");
}


int ledDemoPhase =0; 
int lightupForTap = 0;
unsigned long lastLightupTimeMillis = 0;
unsigned long fastWipeUpdateInterval = 100;//millis
unsigned long slowWipeUpdateInterval = 175;//millis
unsigned long rainbowUpdateInterval = 40;//millis
unsigned long lastDataAcqMillis = 0;
unsigned long tapWipeUpdateInterval = 20;//millis


int wipeCurrentPixel = 0;
int rainbowCycleNumber = 0;

void loop() {
  unsigned long currentMillis = millis();

  // Keep polling for new data from BLE
  ble113.checkActivity();
  
  // blink FemtoduinoBLE Data LED based on state:
  //  - solid = STANDBY
  //  - 1 pulse per second = ADVERTISING
  //  - 2 pulses per second = CONNECTED_SLAVE
  //  - 3 pulses per second = CONNECTED_SLAVE w/ encryption
  uint16_t slice = millis() % 1000;
  if (ble_state == BLE_STATE_STANDBY) {

    digitalWrite(LED_PIN, HIGH);
  } else if (ble_state == BLE_STATE_ADVERTISING) {

    digitalWrite(LED_PIN, slice < 100);
  } else if (ble_state == BLE_STATE_CONNECTED_SLAVE) {

      if(!ble_encrypted) {
        digitalWrite(LED_PIN, slice < 100 || (slice > 200 && slice < 300));
      } else {
        digitalWrite(LED_PIN, slice < 100 || (slice > 200 && slice < 300) || (slice > 400 && slice < 500));
      }
  }

  if (lightupForTap > 0){

    if ((currentMillis - lastLightupTimeMillis ) > tapWipeUpdateInterval){
      colorWipe(strip.Color(255, 255, 255), wipeCurrentPixel); // Green
      lastLightupTimeMillis = currentMillis;

      wipeCurrentPixel++;
    }

    if (wipeCurrentPixel >= PIXELS){
      lightupForTap--;
      wipeCurrentPixel = 0;
    }
  }else if (ledDemoPhase == 0 && (currentMillis - lastLightupTimeMillis ) > fastWipeUpdateInterval){
    colorWipe(strip.Color(0, 255, 0), wipeCurrentPixel); // Green
    lastLightupTimeMillis = currentMillis;

    wipeCurrentPixel++;

    if (wipeCurrentPixel >= PIXELS){
      ledDemoPhase++;
      wipeCurrentPixel = 0;
    }

  }else if (ledDemoPhase == 1 && (currentMillis - lastLightupTimeMillis ) > rainbowUpdateInterval){
    rainbow(rainbowCycleNumber);
    lastLightupTimeMillis = currentMillis;

    rainbowCycleNumber++;

    if (rainbowCycleNumber >= 255){
      ledDemoPhase++;
      rainbowCycleNumber = 0;
    }

  }else if (ledDemoPhase == 2 && (currentMillis - lastLightupTimeMillis ) > slowWipeUpdateInterval){
    colorWipe(strip.Color(255, 0, 0), wipeCurrentPixel); // red
    lastLightupTimeMillis = currentMillis;

    wipeCurrentPixel++;

    if (wipeCurrentPixel >= PIXELS){
      ledDemoPhase++;
      wipeCurrentPixel = 0;
    }
  }else if (ledDemoPhase == 3 && (currentMillis - lastLightupTimeMillis ) > fastWipeUpdateInterval){
    colorWipe(strip.Color(0, 255, 0), wipeCurrentPixel); // Green
    lastLightupTimeMillis = currentMillis;

    wipeCurrentPixel++;

    if (wipeCurrentPixel >= PIXELS){
      ledDemoPhase++;
      wipeCurrentPixel = 0;
    }

  }else if (ledDemoPhase == 4 && (currentMillis - lastLightupTimeMillis ) > rainbowUpdateInterval){
    rainbow(rainbowCycleNumber);
    lastLightupTimeMillis = currentMillis;

    rainbowCycleNumber++;

    if (rainbowCycleNumber >= 255){
      ledDemoPhase++;
      rainbowCycleNumber = 0;
    }

  }else if (ledDemoPhase == 5 && (currentMillis - lastLightupTimeMillis ) > slowWipeUpdateInterval){
    colorWipe(strip.Color(0, 0, 255), wipeCurrentPixel); // blue
    lastLightupTimeMillis = currentMillis;

    wipeCurrentPixel++;

    if (wipeCurrentPixel >= PIXELS){
      ledDemoPhase++;
      wipeCurrentPixel = 0;
    }

  }

  if (ledDemoPhase > 5){
    ledDemoPhase = 0;
  }



  ///accel
  if ((currentMillis - lastDataAcqMillis ) > 50){
    readRegister(DATAX0, 6, values);
    lastDataAcqMillis = currentMillis;

    //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
    //The X value is stored in values[0] and values[1].
    x = ((int)values[1]<<8)|(int)values[0];
    //The Y value is stored in values[2] and values[3].
    y = ((int)values[3]<<8)|(int)values[2];
    //The Z value is stored in values[4] and values[5].
    z = ((int)values[5]<<8)|(int)values[4];
    
    //Convert the accelerometer value to G's. 
    //With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
    // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
    xg = x * 0.0078;
    yg = y * 0.0078;
    
    double oldZg = zg;
    zg = z * 0.0078;
    
    if (abs(oldZg-zg) > .02){

      Serial.print((float)xg,2);
      Serial.print("g,");
      Serial.print((float)yg,2);
      Serial.print("g,");
      Serial.print((float)zg,2);
      Serial.println("g");

      uint8_t zgInt = int(zg*100) + 100;
      ble113.ble_cmd_attributes_write(ble_bonding,0,1,&zgInt);
      //ble_bonding
    }
  }

  if(tapType > 0)
  {
    if(tapType == 1){
      lightupForTap++;

      Serial.println("SINGLE");
      Serial.print(x);
      Serial.print(',');
      Serial.print(y);
      Serial.print(',');
      Serial.println(z);
    }
    else{
      Serial.println("DOUBLE");
      Serial.print((float)xg,2);
      Serial.print("g,");
      Serial.print((float)yg,2);
      Serial.print("g,");
      Serial.print((float)zg,2);
      Serial.println("g");
    }
    attachInterrupt(1, tap, CHANGE);
    tapType=0;    
  }

}


//neoPixels
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t pixel) {
      strip.setPixelColor(pixel, c);
      strip.show();
}

void rainbow(uint8_t rainbowCycleNumber) {
  uint16_t i;

  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel((i+rainbowCycleNumber) & 255));
  }
  strip.show();

}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void debugLog(String logPiece){\
  //milisecondsSinceProgramStart: "log piece"
  //nice logging, printing to com port attached to computer
  const String seperator = String(": ");
  String toSend = String(millis(), DEC) + seperator + logPiece;
  
  
  //debug in the way most preferred 
  Serial.println(toSend);
} 



// BLE

void femtoSystemBoot(const ble_msg_system_boot_evt_t *msg) {
  
#ifdef DEBUG
        Serial.print("###\tsystem_boot: { ");
        Serial.print("major: "); Serial.print(msg -> major, HEX);
        Serial.print(", minor: "); Serial.print(msg -> minor, HEX);
        Serial.print(", patch: "); Serial.print(msg -> patch, HEX);
        Serial.print(", build: "); Serial.print(msg -> build, HEX);
        Serial.print(", ll_version: "); Serial.print(msg -> ll_version, HEX);
        Serial.print(", protocol_version: "); Serial.print(msg -> protocol_version, HEX);
        Serial.print(", hw: "); Serial.print(msg -> hw, HEX);
        Serial.println(" }");
#endif
  // System boot means module is in standby state.
  
  // set advertisement interval to 200-300ms, use all 
  // advertisement channels. Note: min/max parameters are
  // in units of 625 uSec.
  
  ble113.ble_cmd_gap_set_adv_parameters(320, 480, 7);
  while(ble113.checkActivity(1000));
  
  // USE THE FOLLOWING TO LET THE BLE STACK HANDLE YOUR ADVERTISEMENT PACKETS
  // start advertising general discoverable /undirected connectable
  //ble113.ble_cmd_gap_set_mode(BGLIB_GAP_GENERAL_DISCOVERABLE, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
  //while(ble113.checkActivity(1000));
  
  // USE THE FOLLOWING TO HANDLE YOUR OWN CUSTOM ADVERTISEMENT PACKETS
  // build custom advertisement data
  // default BLE stack value: 0201061107e4ba94c3c9b7cdb09b487a438ae55a19
  uint8 adv_data[] = {
    
      0x02,                                     // Field length
      
      BGLIB_GAP_AD_TYPE_FLAGS,                  // Field type (0x01)
      0x06, // data (0x02 | 0x04 = 0x06, general discoverable + BLE only, no BR+EDR) 
        
      0x11,                                     // Field length
      
      BGLIB_GAP_AD_TYPE_SERVICES_128BIT_ALL,    // Field Type (0x07)
      
      0xe4, 0xba, 0x94, 0xc3, 
      0xc9, 0xb7, 0xcd, 0xb0, 
      0x9b, 0x48, 0x7a, 0x43, 
      0x8a, 0xe5, 0x5a, 0x19
  };
  
  ble113.ble_cmd_gap_set_adv_data(0, 0x15, adv_data);
  while(ble113.checkActivity(1000));
  
  // build custom scan response data (i.e. the Device Name value)
  // default BLE stack value: 140942474c69622055314131502033382e344e4657
  uint8 sr_data[] = {
    
    0x14,                                       // Field length
    
    BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE,       // Field type
    
    // Set the name to "FemtoduinoBLE 00:00:00"
       'M', 'a', 'g', 'n', 'e', 't', 'u', 'i', 'n', 'o', 'B', 'L', 'U', ' ', '0', '0', ':', '0', '0', ':', '0', '0'
    //  1    2    3    4    5    6    7    8    9    10   11   12   13  14   15   16   17   18   19   20   21   22
  };
  
  // get the BLE MAC address
  ble113.ble_cmd_system_address_get();
  while(ble113.checkActivity(1000));
  BGAPI_GET_RESPONSE(r0, ble_msg_system_address_get_rsp_t);
  
  // assign last three bytes of MAC address to add packet friendly name (instead of 00:00:00 above)
  sr_data[16] = (r0 -> address.addr[2] / 0x10) + 48 + ((r0 -> address.addr[2] / 0x10) / 10 * 7); // MAC byte 4 10's digit
  sr_data[17] = (r0 -> address.addr[2] & 0xF)  + 48 + ((r0 -> address.addr[2] & 0xF ) / 10 * 7); // MAC byte 4 1's digit
  sr_data[18] = (r0 -> address.addr[1] / 0x10) + 48 + ((r0 -> address.addr[1] / 0x10) / 10 * 7); // MAC byte 5 10's digit
  sr_data[19] = (r0 -> address.addr[1] & 0xF)  + 48 + ((r0 -> address.addr[1] & 0xF ) / 10 * 7); // MAC byte 5 1's digit
  sr_data[20] = (r0 -> address.addr[0] / 0x10) + 48 + ((r0 -> address.addr[0] / 0x10) / 10 * 7); // MAC byte 6 10's digit
  sr_data[21] = (r0 -> address.addr[0] & 0xF)  + 48 + ((r0 -> address.addr[0] & 0xF ) / 10 * 7); // MAC byte 6 1's digit
  
  // set custom scan respnose data (i.e. The Device Name value)
  ble113.ble_cmd_gap_set_adv_data(1, 0x15, sr_data);
  while(ble113.checkActivity(1000));
  
  // put the module into "discoverable/connectable" mode
  // (with user-defined advertisement data)
  ble113.ble_cmd_gap_set_mode(BGLIB_GAP_USER_DATA, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
  while (ble113.checkActivity(1000));
  
  // set state to ADVERTISING
  ble_state = BLE_STATE_ADVERTISING;

}

void femtoConnectionStatus(const ble_msg_connection_status_evt_t * msg) {
#ifdef DEBUG
        Serial.print("###\tconnection_status: { ");
        Serial.print("connection: "); Serial.print(msg -> connection, HEX);
        Serial.print(", flags: "); Serial.print(msg -> flags, HEX);
        Serial.print(", address: ");
        // this is a "bd_addr" data type, which is a 6-byte uint8_t array
        for (uint8_t i = 0; i < 6; i++) {
            if (msg -> address.addr[i] < 16) Serial.write('0');
            Serial.print(msg -> address.addr[i], HEX);
        }
        Serial.print(", address_type: "); Serial.print(msg -> address_type, HEX);
        Serial.print(", conn_interval: "); Serial.print(msg -> conn_interval, HEX);
        Serial.print(", timeout: "); Serial.print(msg -> timeout, HEX);
        Serial.print(", latency: "); Serial.print(msg -> latency, HEX);
        Serial.print(", bonding: "); Serial.print(msg -> bonding, HEX);
        Serial.println(" }");
#endif  
  /**
   * "flags" bit description:
   * - bit 0: connection_connnected
   *          Indicates the connection exists to a remote device.
   * - bit 1: connection_encrypted
   *          Indicates the connection is encrypted.
   * - bit 2: connection_completed
   *          Indicates that a new connection has been created.
   * - bit 3: connection_parameters_change
   *          Indicates that connection parameters have changed, and is 
   *          set when parameters change due to a link layer operation.
   */
   
   // Check to see if a new connection has been established.
   if ((msg -> flags & 0x05) == 0x05) {
     // track state change based on the last known state
     if (ble_state == BLE_STATE_ADVERTISING) {
         ble_state = BLE_STATE_CONNECTED_SLAVE;
     } else {
         ble_state = BLE_STATE_CONNECTED_MASTER;
     }
   }
   
   // update "encrypted" status
   ble_encrypted = msg -> flags & 0x02;
   
   // update "bonded" status
   ble_bonding = msg -> bonding;
}

void femtoConnectionDisconnect(const struct ble_msg_connection_disconnected_evt_t *msg) {
#ifdef DEBUG
        Serial.print("###\tconnection_disconnect: { ");
        Serial.print("connection: "); Serial.print(msg -> connection, HEX);
        Serial.print(", reason: "); Serial.print(msg -> reason, HEX);
        Serial.println(" }");
#endif
  ble113.ble_cmd_gap_set_mode(BGLIB_GAP_USER_DATA, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
  while(ble113.checkActivity(1000));
  
  // set state to ADVERTISING
  ble_state = BLE_STATE_ADVERTISING;
  
  // clear "encrypted" and "bonding" info
  ble_encrypted = 0;
  ble_bonding = 0xFF;
}

void femtoAttributesValue(const struct ble_msg_attributes_value_evt_t *msg) {
  /**
   * @todo Use the msg -> value
   */
   #ifdef DEBUG
   Serial.print("###\tattributes_value: { ");
   Serial.print("connection: "); Serial.print(msg -> connection, HEX);
   Serial.print(", reason: "); Serial.print(msg -> reason, HEX);
   Serial.print(", handle: "); Serial.print(msg -> handle, HEX);
   Serial.print(", offset: "); Serial.print(msg -> offset, HEX);
   Serial.print(", value_len: "); Serial.print(msg -> value.len, HEX);
   Serial.print(", value_data: ");
   // this is a "uint8array" data type, which is a length byte and a uint8_t* pointer
   for (uint8_t i = 0; i < msg -> value.len; i++) {
       if (msg -> value.data[i] < 16) Serial.write('0');
       Serial.print(msg -> value.data[i], HEX);
   }
   Serial.println(" }");
   
   #endif
   
   // Check for data written to "c_rx_data" handle
   if (msg -> handle == GATT_HANDLE_C_RX_DATA ) {
     Serial.println("Got C_RX_DATA handle");
     if (msg -> value.len > 0) {
       
       // set pins 9, 10, 11, and 12 to four lower-most bits of first byte of RX data
       // (nice for controlling RGB LED or something)
       int intVal = (int) msg -> value.data[0];
       
       #ifdef DEBUG
         Serial.print("RX Data:");
         Serial.println(intVal);
       #endif
       
       digitalWrite(RGB_LED_PIN1, intVal & 0x01);
       digitalWrite(RGB_LED_PIN2, intVal & 0x02);
       digitalWrite(RGB_LED_PIN3, intVal & 0x04);
//       digitalWrite(RGB_LED_PIN4, intVal & 0x08);
     }
   }
}


// Event handlers

void onBusy() {
  digitalWrite(LED_PIN, HIGH);
}

void onIdle() {
  digitalWrite(LED_PIN, LOW);
}

void onTimeout() {
    digitalWrite(BLE_RESET_PIN, LOW);
    delay(5);
    digitalWrite(BLE_RESET_PIN, HIGH);
}

void onBeforeTXCommand() {
  // wake module up
  digitalWrite(BLE_WAKEUP_PIN, HIGH);
  
  // wait for "hardware_io_port_status" event to come throgh, and parse it (and otherwise ignore it)
  uint8_t *last;
  while(1) {
    ble113.checkActivity();
    last = ble113.getLastEvent();
    if (last[0] == 0x07 && last[1] == 0x00) break;
  }
  
  // give a bit of a gab between parsing the wake-up event and allowing the 
  // command to go out
  delayMicroseconds(1000);
}

void onTXCommandComplete() {
  // allow module to return to sleep
  digitalWrite(BLE_WAKEUP_PIN, LOW);
}

//accel and tap interrupts

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

// //This function will read a certain number of registers starting from a specified address and store their values in a buffer.
// //Parameters:
// //  char registerAddress - The register addresse to start the read sequence from.
// //  int numBytes - The number of registers that should be read.
// //  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

void tap(void){
  // detachInterrupt(1);
  //Clear the interrupts on the ADXL345
  debugLog("tapped!");
  readRegister(INT_SOURCE, 1, values); 
  if(values[0] & (1<<5))tapType=2;
  else tapType=1;;
}
