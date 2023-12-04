/* 
 * This code is for the ESP32 microcontroller. 
 * It uses 3 HC-SR04 ultrasonic sensors to measure distance and then uses a Kalman filter to smooth the measurements.
 * The distance is then displayed using 4 LEDs and a servo motor.
 
 * HC-SR04 has range of 2-400 cm. Baud rate of 9600.
 
 * Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33 
 * Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
 * Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
 * Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21


 * GPOI that dont work for led: 35, 34, 39, 36, 1
 * GPOI that work for led: 32, 33, 25,      22,23,21, 19, 18 ,26, 27, 14, 12, 13, 9

*/



// Libraries used++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The ESP32Servo library is from: https://registry.platformio.org/libraries/madhephaestus/ESP32Servo
#include <ESP32Servo.h>   // projects/kalmanFilter_with_hc-sr04/kalmanFilter_hc-sr04_ESP32_arduinoFramework/.pio/libdeps/denky32/ESP32Servo/examples
#include <Arduino.h>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// My Kalman filter
#include "KalmanFilter.h"


// All of my variables
#include "variables.h"




// Setup the pins and serial monitor
void setup() {
  for (int i = 0; i < num_sensors; i++) {
    pinMode(ultrasonic[i][0], OUTPUT);
    pinMode(ultrasonic[i][1], INPUT);
  }
  for (int i = 0; i < num_leds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }


	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2500); 	// default min/max of 1000us and 2000us


  Serial.begin(9600);
}

// Measure distance for a sensor
double usonic_transmit(int trig, int echo) {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  double duration = pulseIn(echo, HIGH);

  double distance = 0;
  if (cmORin == 0)
    distance = (duration/2)/73.9;  // Convert to inches
  else
    distance = (duration/2)/29.1;   // Convert to cm

  return distance;
}

// Send the measurements back to the computer for analysis
void sendMeasurements(double measurements[], int size) {

  if (useKalmanFilter)
    Serial.println("Measurements with Kalman filter: ");
  else
    Serial.println("Measurements with out Kalman filter: ");


  for (int i = 0; i < size; i++) {
    Serial.print(measurements[i]);
    Serial.print(", ");
  }
  Serial.println();
}


void moveServo(double distance, double in_min, long in_max, long out_min, long out_max) {
  int position = map(distance, in_min, in_max, out_min, out_max);
  myservo.write(position);
}


// Main loop:
// Measure the distance for each sensor and store the measurements in an array
void loop() {
  // Resetting the variables
  double aggregateD = 0;
  startTime = millis();
  elapsedTime = 0;
  measurementsIndex = 0;
  int position;   // to move the servo


  // Run the loop for 10 seconds then send the measurements back to the computer for analysis
  while (elapsedTime < 10000 && measurementsIndex < 1000) {
    
    // Measure each sensor
    for (int i = 0; i < num_sensors; i++) {
      
      // Measure the distance for one sensor
      double dis = usonic_transmit(ultrasonic[i][0], ultrasonic[i][1]);

      // Check if the distance is within an acceptable range (adjust as needed)
      if (dis >= min_distance[cmORin] && dis <= max_distance[cmORin]) {
        
        // Check if using kalman filter
        if (useKalmanFilter) {
          double estimated_value = KalmanFilter.updateEstimate(dis);
        }

        else{
          aggregateD += dis;  // Used for the 3 sensor averaging method
        }
      }

      else {
        Serial.print("Outlier detected. Skipping Kalman filter update: ");
        Serial.println(dis);
      }
    }
  

    // Find the distance for with or without Kalman filter
    if (useKalmanFilter) {
      aggregateD = KalmanFilter.returnCurrentEstimate();
      }
    else{
      aggregateD = aggregateD / num_sensors;    // Calculate using the e sensor averaging method
    }

    // Store the distance in the measurements array
    measurements[measurementsIndex++] = aggregateD;

    // Print the distance to the Serial Monitor
    if (useKalmanFilter)
      Serial.print("Aggregate distance (Kalman) ");
    else
      Serial.print("Aggregate distance (3avg) ");
    
    Serial.print(units[cmORin]);
    Serial.print(": ");
    Serial.print(aggregateD);
    Serial.print("    elapsedTime (ms): ");
    Serial.println(elapsedTime);





    // Display the distance using the LED
    // loop through ledPins and turn on the leds depending on the distance. turn one more led on for each 10 units
    for (int i = 0; i < num_leds; i++) {
      if (aggregateD > 20 * (i + 1)) {
        digitalWrite(ledPins[i], HIGH);
      }
      else {
        digitalWrite(ledPins[i], LOW);
      }
    }

    // Move the servo depending on the distance
    moveServo(aggregateD, min_distance[cmORin], 80, 0, 70);       // This is just for visualisation really

    delay(100);   // make sure the servo has moved
    elapsedTime = millis() - startTime;
  }

  // Send the measurements back to the computer for analysis
  sendMeasurements(measurements, measurementsIndex);
}