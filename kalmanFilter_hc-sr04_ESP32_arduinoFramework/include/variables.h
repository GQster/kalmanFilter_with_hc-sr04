
// Settings:
bool useKalmanFilter = true;        // Variable to control whether to use the Kalman filter
const int inORcm = 1;               // 0 for inches, 1 for cm
const int num_sensors = 3;
const int num_leds = 4;


// Constants:
const int min_distance[2] = {1, 2}; // Minimum distance in inches/cm
const int max_distance[2] = {160, 400}; // Maximum distance in inches/cm
const char* units[2] = {"(inch)", "(cm)"};


// Pin out************************************************************
int servoPin = 33;
const int ledPins[4] = {19, 21, 22, 23};        
// 2D array to store [trig, echo] pins of three ultrasonic sensors
const int ultrasonic[3][2] = {
  {13, 9},
  {14, 12},
  {26, 27}
};
// ********************************************************************



// Objects *************************
Servo myservo;                              // create servo object to control a servo
KalmanFilter KalmanFilter(2, 2, 0.01);      // Measurement err, estimate err, process noise
// *********************************



// To send the data back after the loop as an array********************************
unsigned long startTime;  // Variable to store the start time of each phase
unsigned long elapsedTime; // Variable to store the elapsed time
double measurements[1000];  // Adjust the size as needed
int measurementsIndex = 0;
// *********************************************************************************

