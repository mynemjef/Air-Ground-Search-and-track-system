#include <math.h>
#include <ESP32Servo.h>
#include "LIDARLite_v4LED.h"

LIDARLite_v4LED myLIDAR;

// Define servo pins (Beta is the vertically turning servo motor)
#define SERVO_PIN_ALPHA 3
#define SERVO_PIN_BETA  5

// Define lidar measurements per second and time to be simulated
#define tick_rate           15  
float cycle_duration_ms = 1000/tick_rate;

// Define max lidar detection range for very large target in meters, divergence in degrees and area to be scanned in degrees
// Set vertical scan to the desired furthest up, Set horizontal scan to the desired horizontal degrees
#define max_lidar_range             250
#define lidar_divergence            5
#define vertical_degrees_scanned    20
#define horizontal_degrees_scanned  60

// Define the times acquiring needs to not find a target before acquisition is canceled and the size of the starting acquisition box is in degrees (The box changes size dynamically based on target size)
#define acq_width     10
#define acq_height    6
#define ttl_scans     5

// Variables used for keeping track of the TTL of the acquisition and the dynamic acquisition box
int high_acq_edge, low_acq_edge, left_acq_edge, right_acq_edge;
int ttl;

// Arrays to store X, Y, Z coordinates of the target at two different points in time, used for data calculation
float first_target_coordinates[3];
float second_target_coordinates[3];

// Variables for target data
float target_speed, old_target_speed, acceleration, heading;

// First bool is used for alternating between which X, Y, Z coordinate array is being written to, second represents whether the system is tracking a target
bool first_scan = true;
bool acquiring = false;

// Calculate largest area that can be scanned without leaving areas unscanned
float bar_size = (lidar_divergence / 2) * sqrt(2);
float vertical_bar_size = bar_size;

// Define values used for servo guidance in degrees (-90 to +90 for horizontal, 0 to +90 for vertical)
float alpha_servo_degrees = 0, beta_servo_degrees = 0;
int alpha_servo_bars = 0, beta_servo_bars = 0;


// Initialize servo motors
Servo servoAlpha;
Servo servoBeta;

// 2D array for storing distances to ground clutter for filtration
// IMPORTANT: SIZE MUST BE PRECALCULATED BASED ON DIVERGENCE SO THAT IT ISN'T TOO SMALL
float ground_clutter[36][18];

// Sensor's maximum error in meters. Used to prevent returning clutter as a target due to imprecision in separate measurements.
float deviation = 0.5;

//Total cycles made
int current_cycle = 0;
int nextCycleMillis;

// Bool used to keep track of vertical servo motor's rotation direction
bool seeking_up = false;

// Function to calculate the next cycle's millis
int calculateNextCycleMillis() {
  nextCycleMillis = cycle_duration_ms * current_cycle;
  return nextCycleMillis;
}

void printServoDebugInfo() {
  Serial.print(alpha_servo_degrees);
  Serial.print(" / ");
  Serial.println(beta_servo_degrees);
}

// Void to move servos across search area without leaving spots unscanned one movement at a time.
void moveServos(){
  servoAlpha.write(alpha_servo_degrees);
  servoBeta.write(beta_servo_degrees);
  
  alpha_servo_bars = round(alpha_servo_degrees / abs(bar_size));
  beta_servo_bars = round(beta_servo_degrees / abs(bar_size));
 
  alpha_servo_degrees += bar_size;

  if (alpha_servo_degrees >= horizontal_degrees_scanned) {
    bar_size = -bar_size;
    beta_servo_degrees += vertical_bar_size;
  } else if (alpha_servo_degrees <= 0) {
    bar_size = abs(bar_size);
    beta_servo_degrees += vertical_bar_size;
  }
  if (beta_servo_degrees >= vertical_degrees_scanned) {
    vertical_bar_size = -abs(vertical_bar_size);
    if (seeking_up){
      Serial.println();
      seeking_up = !seeking_up;
    }
  } else if (beta_servo_degrees <= 0) {
    if (!seeking_up){
      Serial.println();
      seeking_up = !seeking_up;
    }
    vertical_bar_size = abs(vertical_bar_size);
  }
}

float lidarMeasurement(){
  float newDistance;
  newDistance = myLIDAR.getDistance();
  newDistance = newDistance / 100;
  return newDistance;
}


float calculateHeight(float distance) {
  // Convert angles from degrees to radians
  float verticalAngleRad = radians(beta_servo_degrees);

  // Calculate the height (Z-coordinate) of the point
  float height = distance * tan(verticalAngleRad);

  return height;
}

void targetAcquisition(){
  int high_target_edge = 0, low_target_edge = 0, left_target_edge = left_acq_edge, right_target_edge = 0;
  for (int y_degrees = low_acq_edge; y_degrees <= high_acq_edge; y_degrees += bar_size){
    for (int x_degrees = left_acq_edge; x_degrees <= right_acq_edge; x_degrees += bar_size){
      servoAlpha.write(x_degrees);
      servoBeta.write(y_degrees);
      float lidar_distance_to_target = lidarMeasurement();
      if (lidar_distance_to_target < max_lidar_range){
        ttl = ttl_scans;
        low_target_edge = y_degrees;
        if(right_target_edge < x_degrees){
          right_target_edge = x_degrees;
        }
        if(high_target_edge < y_degrees){
          high_target_edge = y_degrees;
        }
        if(left_target_edge > x_degrees){
          high_target_edge = x_degrees;
        }
        if (first_scan){
          first_scan = !first_scan;
          first_target_coordinates[0] = lidar_distance_to_target * cos(x_degrees) * sin(y_degrees);
          first_target_coordinates[1] = calculateHeight(lidar_distance_to_target);
          first_target_coordinates[2] = lidar_distance_to_target * sin(x_degrees) * sin(y_degrees);
        }
        else{
          first_scan = !first_scan;
          second_target_coordinates[0] = lidar_distance_to_target * cos(x_degrees) * sin(y_degrees);
          second_target_coordinates[1] = calculateHeight(lidar_distance_to_target);
          second_target_coordinates[2] = lidar_distance_to_target * sin(x_degrees) * sin(y_degrees);
        }
      }
      
      old_target_speed = target_speed;
      target_speed = sqrt(sq(second_target_coordinates[0] - first_target_coordinates[0]) + sq(second_target_coordinates[1] - first_target_coordinates[1]) + sq(second_target_coordinates[2] - first_target_coordinates[2])) / cycle_duration_ms;
      
      acceleration = target_speed - old_target_speed;
      
      heading = atan((second_target_coordinates[2] - first_target_coordinates[2]) / (second_target_coordinates[0] - first_target_coordinates[0]));
      if ((second_target_coordinates[2] - first_target_coordinates[2]) < 0 && (second_target_coordinates[0] - first_target_coordinates[0]) >= 0) {
        heading += PI;
      } else if ((second_target_coordinates[2] - first_target_coordinates[2]) < 0 && (second_target_coordinates[0] - first_target_coordinates[0]) < 0) {
        heading -= PI;
      }
      
      Serial.print("Speed: ");
      Serial.print(target_speed);
      Serial.print(" Height: ");
      Serial.print(calculateHeight(lidar_distance_to_target));
      Serial.print(" Acceleration: ");
      Serial.print(acceleration);
      Serial.print(" Heading: ");
      Serial.println(degrees(heading));
    }

    high_acq_edge = high_target_edge + bar_size;
    low_acq_edge = low_target_edge - bar_size;
    left_acq_edge = left_target_edge - bar_size;
    right_acq_edge = right_target_edge + bar_size;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  servoAlpha.attach(SERVO_PIN_ALPHA);
  servoBeta.attach(SERVO_PIN_BETA);
  if (myLIDAR.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("LIDAR acknowledged!");
  
  // Initial scan for ground clutter filtration
  while (beta_servo_degrees <= vertical_degrees_scanned || alpha_servo_degrees <= horizontal_degrees_scanned){
    moveServos();
    ground_clutter[alpha_servo_bars][beta_servo_bars] = lidarMeasurement();
    delay(cycle_duration_ms);
  }
}

void loop() {
  // Wait until it's time to start the cycle
  while (millis() <= calculateNextCycleMillis()) {}
    
  if (Serial.available() >= 5) {
    if (!acquiring){
      ttl = ttl_scans;
      acquiring = !acquiring;
      low_acq_edge = Serial.parseInt() - (acq_height/2);
      Serial.read();
      left_acq_edge = Serial.parseInt() - (acq_width/2);
      Serial.read();
      right_acq_edge = left_acq_edge + acq_width;
      high_acq_edge = low_acq_edge + acq_height;
    } else {acquiring = !acquiring;}
  }

  if (ttl > 0 && acquiring){
    ttl--;
    targetAcquisition();
  }else {
    moveServos();
    float lidar_distance_to_target = lidarMeasurement();
    if(ground_clutter[alpha_servo_bars][beta_servo_bars] <= lidar_distance_to_target + deviation){
      lidar_distance_to_target = max_lidar_range;
    }
    if (lidar_distance_to_target < max_lidar_range){
      Serial.print(lidar_distance_to_target);
      Serial.print(",");
    }else {Serial.print("0");}
  }

  //printServoDebugInfo();
  //printTargetDebugInfo();
  //Serial.println(millis());
  current_cycle += 1;
}
