/*  
    Group34 - Over-Speed Detection
*/

#include <Wire.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

double startTime = 0; //Initialize the start time
double endTime = 0;  //Initialize the end time
double timeTakenInSeconds = 0; //Difference between start and end time
double speedOfObject = 0;  //Holds the value distance divided by time taken
const double distance = 0.2;  // Distance between the two sensors is set to 1 metre
int executed = 0;   //Flag to run the code in the loop only once. When set to 1, code in the loop is not executed.
int sensor1 = 10;    //First sensor is connected to pin 9
int sensor2 = 8;   //Second sensor is connected to pin 10
int led1 = 12;   //An LED to indicate that the first sensor has sensed an object
int led2 = 13;   //An LED to indicate that the second sensor has sensed an object
int buzzer = 11;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


void setup() {

  Serial.begin(9600);
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

  lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("GROUP 34 ");
    lcd.setCursor(0, 1);
    lcd.print("Speed  Detection");
    delay(2000);
    lcd.clear();

}

void loop() {
  
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
        displaySpeed(speedOfObject); // Display the speed on the LCD
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
        displaySpeed(speedOfObject); // Display the speed on the LCD
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
    delay(100);
  }
}
void displaySpeed(double speedOfObject) {
  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set the cursor to the top-left

  // Display the speed value
  // lcd.print("The speed is: ");
  lcd.print(speedOfObject, 2); // Displaying with 2 decimal places
  lcd.print(" km/h");

  lcd.setCursor(0, 1); // Move to the next line

  // Display the message based on speed
  if (speedOfObject > 30) {
    lcd.print("Over speed !");
  } else {
    lcd.print("Normal speed");
  }
}


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


