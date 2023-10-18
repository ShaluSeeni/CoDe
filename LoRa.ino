/*  ******
    Group34 - Over-Speed Detection
*/

#include <Wire.h>
#include <LiquidCrystal.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

double startTime = 0; //Initialize the start time
double endTime = 0;  //Initialize the end time
double timeTakenInSeconds = 0; //Difference between start and end time
// double speedOfObject = 0;  //Holds the value distance divided by time taken
const double distance = 0.2;  // Distance between the two sensors is set to 1 metre
int executed = 0;   //Flag to run the code in the loop only once. When set to 1, code in the loop is not executed.
int sensor1 = 10;    //First sensor is connected to pin 9
int sensor2 = 11;   //Second sensor is connected to pin 10
int led1 = 12;   //An LED to indicate that the first sensor has sensed an object
int led2 = 13;   //An LED to indicate that the second sensor has sensed an object
int buzzer = 3;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x37, 0x50, 0x1C, 0x3B, 0xF7, 0xB5, 0x3C, 0xDE, 0xB0, 0xAB, 0x9E, 0x92, 0x7B, 0xA3, 0x12, 0x28 }; 

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0xBF, 0x19, 0xA4, 0x93, 0x80, 0x42, 0x78, 0x58, 0x3C, 0xD2, 0x07, 0xC1, 0x83, 0x98, 0x34, 0x4E }; 

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260DC137;


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


char mydata[] = "Hello, world!!!!!!";
double speedOfObject = 0;
byte downlink_active = 0;
byte newdata[1];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

int RX_LED= 12;

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) 
            {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload: "));
              for (int i = 0; i < LMIC.dataLen; i++) 
              {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) 
                {
                  Serial.print(F("0"));
                }
                Serial.println(LMIC.frame[LMIC.dataBeg + i], HEX);
                uint8_t byte_recieved = LMIC.frame[LMIC.dataBeg + i];
                if(byte_recieved == 208 && i==0)
                {
                  Serial.println(F("Making LED blink once."));
                  digitalWrite(RX_LED, HIGH);
                  delay(1000);
                  digitalWrite(RX_LED, LOW);      
                }
                else if(byte_recieved == 209 && i==0)
                {
                  Serial.println(F("Making LED blink twice."));
                  digitalWrite(RX_LED, HIGH);
                  delay(1000);
                  digitalWrite(RX_LED, LOW);
                  delay(1000);
                  digitalWrite(RX_LED, HIGH);
                  delay(1000);
                  digitalWrite(RX_LED, LOW);
                }
                if(i<1)
                  newdata[i] = byte_recieved + 1;
                downlink_active = 1;
              }
              
//              Serial.print("txCnt :"); Serial.println(LMIC.txCnt);
//              Serial.print("txrxFlags :"); Serial.println(LMIC.txrxFlags);
//              Serial.print("dataBeg :"); Serial.println(LMIC.dataBeg);
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Determine which data to send
        uint8_t* dataToSend;
        uint8_t dataLength;
        if (downlink_active == 0) {
            dataToSend = (uint8_t*)mydata;
            dataLength = strlen(mydata); // Assuming it's a null-terminated string
        } else {
            dataToSend = newdata;
            dataLength = sizeof(newdata);
        }

        // Send the data
        LMIC_setTxData2(1, dataToSend, dataLength, 0);

        // Debugging: print the data and frequency
        Serial.print(F("Sending data: "));
        for (int i = 0; i < dataLength; i++) {
            Serial.print((char)dataToSend[i]);
        }
        Serial.println();
        Serial.print(F("Packet queued for frequency (Hz): "));
        Serial.println(LMIC.freq);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}



void setup() {
  while (!Serial);
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Starting"));

  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif

  os_init();
  LMIC_reset();
// Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. The LMIC doesn't let you change
    // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

    // ... extra definitions for channels 2..n here
    #elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

    // ... extra definitions for channels 3..n here.
    #else
    # error Region not supported
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);



  //Sets the pin modes of sensors and LEDs
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  
  //LEDs are turned off
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);

  delay(1000);

  // lcd.begin(16, 2);
  //   lcd.clear();
  //   lcd.setCursor(0, 0);
  //   lcd.print(" GROUP 34 ");
  //   lcd.setCursor(0, 1);
  //   lcd.print("Speed  Detection");
  //   delay(2000);
  //   lcd.clear();

}

void loop() {
  
  // Serial.println();
  // Serial.print("SENSOR 1: ");
  // Serial.print(digitalRead(sensor1));
  // Serial.println();
  // Serial.print("SENSOR 2: ");
  // Serial.print(digitalRead(sensor2));
  // Serial.println();
  unsigned long now;
    now = millis();
    if ((now & 512) != 0) {
      digitalWrite(13, HIGH);
    }
    else {
      digitalWrite(13, LOW);
    }

    os_runloop_once();

  if (executed == 0) { //Flag is initialized to 0 to ensure that the loop runs only once

    if (digitalRead(sensor1) == HIGH) { //If movement is detected by the first sensor
      

      digitalWrite(led1, HIGH); //Turn on the LED
      // digitalWrite(led_signal, LOW);

      if (startTime == 0) { //If startTime is 0 and no time has been logged yet

        startTime = millis(); //Assign Arduino time to startTime

      }

    }

    if (digitalRead(sensor2) == HIGH) { //If movement is detected by the second sensor

      digitalWrite(led2, HIGH); //Turn on the LED
      // digitalWrite(led_signal, LOW);


      if (endTime == 0) {  //If endTime is 0 and no time has been logged yet

        endTime = millis();   //Assign Arduino time to endTime

      }

    }

    if ((startTime != 0) && (endTime != 0)) { //Now calculate the speed of the object if both times have been registered

      if (startTime < endTime) { //If the object moves from sensor on pin 9 to sensor on pin 11

        timeTakenInSeconds = (endTime - startTime) / 1000.0; //Convert milliseconds to seconds
        speedOfObject = (distance / timeTakenInSeconds) * 3.6; //Calculate speed in metres per second
        // displaySpeed(speedOfObject); // Display the speed on the LCD
        executed = 1;
        // reset();
        
      

        if (speedOfObject > 5){
          soundBuzzer();
        }

        /* Print the values to the serial monitor */
        Serial.print("Start Time: "); //Print to the serial monitor
        Serial.print(startTime);   //Print value
        Serial.println(" milliseconds");
        Serial.print("End Time: ");   //Print value
        Serial.print(endTime);    //Print value
        Serial.println(" milliseconds");
        Serial.print("Speed of Object = ");
        Serial.print(speedOfObject);
        Serial.println(" km/h");
        Serial.println(" ");


        executed = 1; //Set the value to 1 to stop the loop from running again
        reset();

      }

      else {     //If the object moves from sensor on pin 11 to sensor on pin 9

        timeTakenInSeconds = (startTime - endTime) / 1000.0; //The value of startTime is greater than endTime
        speedOfObject = (distance / timeTakenInSeconds) * 3.6;  //Calculate speed in metres per second
        //displaySpeed(speedOfObject); // Display the speed on the LCD
        executed = 1;
        // reset();

        if (speedOfObject > 5){
          soundBuzzer();
        }


        /* Print the values to the serial monitor */
        Serial.print("Start Time: ");
        Serial.print(endTime);
        Serial.println(" milliseconds");
        Serial.print("End Time: ");
        Serial.print(startTime);
        Serial.println(" milliseconds");
        Serial.print("Speed of Object = ");
        Serial.print(speedOfObject);
        Serial.println(" km/h");
        Serial.println(" ");


        /* Print the values on the OLED Display */
        executed = 1;   //Set the value to 1 to stop the loop from running again
        reset();  //A function to start the loop from running again

      }
    }
  }
  if (startTime == 0 && endTime ==0) {
    //Serial.println("################");
    delay(200);
  }  
  if (speedOfObject != 0.0) {
    // Update mydata with the value of speedOfObject
    int speedWhole = (int)speedOfObject;  
    int speedFractional = (int)((speedOfObject - speedWhole) * 100);   
    snprintf(mydata, sizeof(mydata), "Speed: %d.%02d km/h", speedWhole, speedFractional);

    // Print mydata to the Serial monitor for debugging
    Serial.println(mydata);

    // Now transmit the updated data using LMIC
    if (!(LMIC.opmode & OP_TXRXPEND)) {
        do_send(&sendjob);
        Serial.println("Calling do_send()");
        speedOfObject = 0.0; // Reset the speed to prevent duplicate sends
    }
}

  
}


// void displaySpeed(double speedOfObject) {
//   lcd.clear(); // Clear the LCD screen
//   lcd.setCursor(0, 0); // Set the cursor to the top-left

//   // Display the speed value
//   // lcd.print("The speed is: ");
//   lcd.print(speedOfObject, 2); // Displaying with 2 decimal places
//   lcd.print(" km/h");

//   lcd.setCursor(0, 1); // Move to the next line

//   // Display the message based on speed
//   if (speedOfObject > 30) {
//     lcd.print("Over speed !");
//   } else {
//     lcd.print("Normal speed");
//   }
// }


void soundBuzzer() {
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);
  }


void reset() {

  delay(3000);
  // digitalWrite(buzzer, HIGH);
  // delay(100);
  // digitalWrite(buzzer, LOW);
  executed = 0;
  startTime = 0;
  endTime = 0;    //Set the value to 0 to start the loop from running again
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);  //Turns off both the LEDs
  speedOfObject = 0;    //sets the speed to 0
  // digitalWrite(led_signal, HIGH);
}

void updateMyData() {
    snprintf(mydata, sizeof(mydata), "Speed: %.2f", speedOfObject);
}



