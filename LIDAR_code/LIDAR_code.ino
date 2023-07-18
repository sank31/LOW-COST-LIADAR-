

//Libraries
#include <Wire.h>       
#include <VL53L0X.h>        //Downlaod it here: https://www.electronoobs.com/eng_arduino_Adafruit_VL53L0X.php


VL53L0X sensor;             //Define our sensor
//If you uncomment any of lines below you activate that mode
#define LONG_RANGE
#define HIGH_SPEED
//#define HIGH_ACCURACY



//Outputs/inputs
#define dirPin 6      //Pin for direction of the stepper driver
#define stepPin 4     //Pin for steps of the stepper driver
#define Enable 5      //Pin for enable the stepper driver

//Variables
int Value = 1000;               //Delay value between steps
float angle = 0;                //Start angle

/* ----------------Step angle calculation----------------
 * We need 1.5 rotations for 360º. (pully ratio 1.5 : 1)
 * Each 200 steps the motor will make a rotation.
 * We move 2 steps and the we make a measurement. 
 * This equals to 360º/(200steps * 1.5) * 2 = 2.4angle/loop  -> 
   ----------------Step angle calculation----------------*/
//float angle_step = 2.4;             //So place that value here    
float angle_step = 3.72;             //So place that value here    
              
float maxdist = 400;                //I've set the maximum distance around the sensor to only 400mm. Change to any other value. 
bool loop_starts = false; 
byte last_PIN_state;


void setup() {
    // Declare pins as output:
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(Enable, OUTPUT);
    digitalWrite(Enable,HIGH);      //Place enable to low so the driver is enabeled
    digitalWrite(dirPin, LOW);      //Place dirPin to HIGH so we spin CW
    Serial.begin(9600);             //Start serial port
    Wire.begin();
    sensor.init();
    sensor.setTimeout(500);
    PCICR |= (1 << PCIE0);          //enable PCMSK0 scan so we can use interrupts                                                
    
                                    //Set pin "D8" trigger an interrupt on "any" state change.
    //PCMSK0 |= (1 << PCINT0);      // Arduino UNO, Nano, Pro Mini, etc. with Atmega328 
    PCMSK0 |= (1 << PCINT4);        // Arduino Micro with Atmega32u4
                                    //See interrupt vector below the void loop                                          
    
    
    
    #if defined LONG_RANGE
        // lower the return signal rate limit (default is 0.25 MCPS)
        sensor.setSignalRateLimit(0.1);
        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);       
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif
    #if defined HIGH_SPEED
        // reduce timing budget to 20 ms (default is about 33 ms)
        sensor.setMeasurementTimingBudget(20000);
    #elif defined HIGH_ACCURACY
        //  // increase timing budget to 200 ms
        //  sensor.setMeasurementTimingBudget(200000);
    #endif  
}//End setup loop



void loop() {
    if (loop_starts)            //We reset angle when the magnet is detected on D8
    {
        angle = 0;
        loop_starts = false;
    }  

    digitalWrite(stepPin, HIGH);                    //Make one step
    delayMicroseconds(Value);                       //Small delay
    digitalWrite(stepPin, LOW);                     //Make another step
    //delayMicroseconds(Value);                       //Add another delay

    int r = sensor.readRangeSingleMillimeters();    //Get distance from sensor
    if (r > maxdist)                                //Limit the dsitance to maximum set distance above
    {
        r = maxdist;
    }
    Serial.print(angle);          //Print the values to serial port
    Serial.print(",");
    Serial.print(r);
    Serial.println(",");
    
    angle = angle + angle_step;   //Increase angle value by the angle/loop value set above (in this case 2.4º each loop)

}//end of void loop



//This is the magnet detection interruption routine
//----------------------------------------------

ISR(PCINT0_vect){
    //We make an AND with the pin state register, We verify if pin 8 is HIGH???
    //if(PINB & B00000001)        // Arduino UNO, Nano, Pro Mini, etc. with Atmega328 
    if(PINB & B00010000)          // Arduino Micro with Atmega32u4 
    {        
        if(last_PIN_state == 0)
        {
            last_PIN_state = 1;                                     
        }
    }
    else if(last_PIN_state == 1)  //Now verify if pin 8 is LOW??? -> Magnet was detected
    {                               
        last_PIN_state = 0;  
        loop_starts = true;         //If yes, we set loop_starts to true so we reset the angle value                                                           
    }
}//End of ISR
