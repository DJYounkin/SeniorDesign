#include <Bounce2.h>
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 2);
unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len  = 0;
unsigned int HighByte2 = 0;
unsigned int LowByte2  = 0;
unsigned int Len2  = 0;
unsigned int DistanceVal = 0;

int redPin = 9;
int greenPin = 10;
int bluePin = 11;

#define PIN_PMETER          A5    // PIN Potentiometer
#define PIN_PUSHERESC       3     // PIN to control ESC, normally the white wire from ESC 
#define PIN_BTN             7     // PIN for button to On/Off reading from Potentiometer

/*///////////////////////////////////////////////////////////////////////////////////////////////////////
 * The following are the setting for the ESC used, for other ESC, just chnage setting according to
 * ESC Spec
 *///////////////////////////////////////////////////////////////////////////////////////////////////////
#define THROTTLE_MIN        1500
#define THROTTLE_MAX        2000
#define THROTTLE_BAKE       1000
// End of ESC setting ///////////////////////////////////////////////////////////////////////////////////

int     throttle      = THROTTLE_MIN; // default throttle
boolean startReading  = false;        // default to not reading 

Servo   pusherESC;                    
Bounce  btnStart      = Bounce(); 

void setLEDColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}





void setup() {
  Serial.begin(9600);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  mySerial.begin(9600);
  Serial.println("Started Setup........");

  pinMode(PIN_BTN, INPUT_PULLUP);   // Using Internal PULLUP resistor
  btnStart.attach(PIN_BTN);
  btnStart.interval(5);
  
  pusherESC.attach(PIN_PUSHERESC);
  pusherESC.writeMicroseconds(THROTTLE_MIN);
  Serial.println("Arming........");   // just some display message 
  delay(2000);
  Serial.println("Arming........After delay");  

  // when running arduino without computer, after seeing the onboard LED goes on and then off
  // I will know the setup had completed and the arduino is now in the loop 
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  btnStart.update(); // update the button status

  if (btnStart.fell()) { // button pressed
    if (!startReading) {   // if not in reading mode
      startReading = true; // change to reading mode
      digitalWrite(LED_BUILTIN, HIGH); // on the onboard LED, for visual indicator  
      Serial.println("Start Sensor Read ----- ");
    } else { // currently in reading mode, should stop reading and brake DC brushed motor
      pusherESC.writeMicroseconds(THROTTLE_BAKE); // brake DC brushed motor
      startReading = false; // change to stop reading mode
      throttle = THROTTLE_BAKE;
      digitalWrite(LED_BUILTIN, LOW); // off the onboard LED, for visual indicator
      Serial.println("Stop Sensor Read ----- ");
      Serial.println("Bake ");
    }
  }

  if (startReading) { // in reading mode
    int sensorValue = analogRead(PIN_PMETER); // update reading from Potentiometer

    int newThrottle  = map(sensorValue, 0, 1023, THROTTLE_MIN, THROTTLE_MAX); 
    int throttleDiff = abs(newThrottle - throttle);

    //if (throttleDiff > 5) { // update ESC only when the reading differ by 5
      throttle = newThrottle;
      Serial.print("Sensor Read : ");
      Serial.print(sensorValue);
      
      Serial.println("Throttle Value : ");
      Serial.println(throttle);
      //pusherESC.writeMicroseconds(throttle); // update ESC
      mySerial.flush();
  mySerial.write(0X55);                           // trig US-100 begin to measure the distance
  //delay(500);                                  
  if (mySerial.available() >= 2)                  // check receive 2 bytes correctly
  {
    
    HighByte = mySerial.read();
    LowByte  = mySerial.read();
    Len  = HighByte * 256 + LowByte;          // Calculate the distance
    if(Len < 250){           //RED
    setLEDColor(255, 0, 0);  // red
    
    DistanceVal = 1500;//1 + 1500;
    digitalWrite(LED_BUILTIN, LOW);
    //delay(100);
    }
  /*else if (250 < Len && Len < 1500)
    DistanceVal = 50+ 1500;*/
    else if (250 <= Len && Len < 2200){   //YELLOW
    setLEDColor(255, 255, 0);  // Orange
    DistanceVal = 100+ 1500;
        digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("Orange");
    //delay(100);
    }
    
  else if (Len > 2200){          //GREEN
     setLEDColor(0, 255, 0);  // green
         digitalWrite(LED_BUILTIN, HIGH);

    DistanceVal = 499 + 1500;
    //delay(100);
  }
    if ((Len > 1) && (Len < 10000))
    {
      Serial.print("Distance1: ");
      Serial.print(Len, DEC);          
      Serial.println("mm");
      Serial.print(DistanceVal, DEC);                  
    }
  }
 pusherESC.writeMicroseconds(DistanceVal);
  //delay(10);  
    //}
  }
  /* dj code below
  mySerial.flush();
  mySerial.write(0X55);                           // trig US-100 begin to measure the distance
  //delay(500);                                  
  if (mySerial.available() >= 2)                  // check receive 2 bytes correctly
  {
    HighByte = mySerial.read();
    LowByte  = mySerial.read();
    Len  = HighByte * 256 + LowByte;          // Calculate the distance
    if(Len < 250){           //RED
    setLEDColor(255, 0, 0);  // red
    DistanceVal = 1000;//1 + 1500;
    //delay(100);
    }
  /*else if (250 < Len && Len < 1500)
    DistanceVal = 50+ 1500;
    else if (250 <= Len && Len < 2200){   //YELLOW
    setLEDColor(255, 255, 0);  // Orange
    DistanceVal = 200+ 1500;
    Serial.println("Orange");
    //delay(100);
    }
    
  else if (Len > 2200){          //GREEN
     setLEDColor(0, 255, 0);  // green
    DistanceVal = 499 + 1500;
    //delay(100);
  }
    if ((Len > 1) && (Len < 10000))
    {
      Serial.print("Distance1: ");
      Serial.print(Len, DEC);          
      Serial.println("mm");
      Serial.print(DistanceVal, DEC);                  
    }
  }
 pusherESC.writeMicroseconds(DistanceVal);
  delay(10);                 */        


  
}
