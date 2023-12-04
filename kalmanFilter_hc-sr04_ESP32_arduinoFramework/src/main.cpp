/* 
 * HC-SR04 has range of 2-400 cm. Baud rate of 9600.
 */


#include <Arduino.h>
#include "KalmanFilter.h"

// Settings:
bool useKalmanFilter = true;        // Variable to control whether to use the Kalman filter
const int cmORin = 0;               // 0 for inches, 1 for cm
const int num_sensors = 3;


// Constants:
const int min_distance[2] = {1, 2}; // Minimum distance in inches/cm
const int max_distance[2] = {160, 400}; // Maximum distance in inches/cm
const char* units[2] = {"(inch)", "(cm)"};

// 2D array to store [trig, echo] pins of three ultrasonic sensors
const int ultrasonic[3][2] = {
  {13, 9},
  {27, 14},
  {32, 33}
};


KalmanFilter KalmanFilter(2, 2, 0.01);      // Measurement err, estimate err, process noise


// To send the data back after the loop as an array
unsigned long startTime;  // Variable to store the start time of each phase
unsigned long elapsedTime; // Variable to store the elapsed time
double measurements[1000];  // Adjust the size as needed
int measurementsIndex = 0;




// Setup the pins and serial monitor
void setup() {
  for (int i = 0; i < num_sensors; i++) {
    pinMode(ultrasonic[i][0], OUTPUT);
    pinMode(ultrasonic[i][1], INPUT);
  }

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





// Main loop:
// Measure the distance for each sensor and store the measurements in an array
void loop() {
  // Resetting the variables
  double aggregateD = 0;
  startTime = millis();
  elapsedTime = 0;
  measurementsIndex = 0;


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

    delay(10);
    elapsedTime = millis() - startTime;
  }

  // Send the measurements back to the computer for analysis
  sendMeasurements(measurements, measurementsIndex);
}