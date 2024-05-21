#include <stdio.h>
#include <math.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"
#include <ctype.h>
#include "soc/mcpwm_periph.h"
#include "LIDARLite_v4LED.h"

LIDARLite_v4LED myLIDAR;

// Define servo pins (Beta is the vertically turning servo motor) and PWM channels
#define SERVO_PIN_ALPHA 3
#define SERVO_PIN_BETA  5
#define SERVO_MIN_PULSEWIDTH 500 // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH 2500 // Maximum pulse width in microseconds
#define SERVO_MAX_DEGREE 180

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

// Calculate largest area that can be scanned without leaving areas unscanned with the formula "(lidar_divergence / 2) * sqrt(2)"
float bar_size = 3.5;
float vertical_bar_size = 3.5;

// Define values used for servo guidance in degrees (-90 to +90 for horizontal, 0 to +90 for vertical)
float alpha_servo_degrees = 0, beta_servo_degrees = 0;
int alpha_servo_bars = 0, beta_servo_bars = 0;

// 2D array for storing distances to ground clutter for filtration
// IMPORTANT: SIZE MUST BE PRECALCULATED BASED ON DIVERGENCE SO THAT IT ISN'T TOO SMALL
float ground_clutter[36][18];

// Sensor's maximum error in meters. Used to prevent returning clutter as a target due to imprecision in separate measurements.
float deviation = 0.5;

//Total cycles made
int current_cycle = 0;
int64_t nextCycleMillis;

size_t buffered_data_length;

// Bool used to keep track of vertical servo motor's rotation direction
bool seeking_up = false;

void init_pwm() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN_ALPHA);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, SERVO_PIN_BETA);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e., for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings
}

void set_servos_degrees(uint32_t angle_servo1, uint32_t angle_servo2) {
    uint32_t duty_servo1, duty_servo2;
    //Convert angle to duty cycle for servo 1
    duty_servo1 = (angle_servo1 / (float)SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) + SERVO_MIN_PULSEWIDTH;
    //Convert angle to duty cycle for servo 2
    duty_servo2 = (angle_servo2 / (float)SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) + SERVO_MIN_PULSEWIDTH;
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_servo1);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_servo2);
}



// Function to calculate the next cycle's millis
int64_t calculateNextCycleMillis() {
  nextCycleMillis = cycle_duration_ms * current_cycle;
  return nextCycleMillis;
}

// Void to move servos across search area without leaving spots unscanned one movement at a time.
void moveServos(){
  set_servos_degrees(alpha_servo_degrees, beta_servo_degrees);
  alpha_servo_bars = round(alpha_servo_degrees / fabs(bar_size));
  beta_servo_bars = round(beta_servo_degrees / fabs(bar_size));
 
  alpha_servo_degrees += bar_size;

  if (alpha_servo_degrees >= horizontal_degrees_scanned) {
    bar_size = -bar_size;
    beta_servo_degrees += vertical_bar_size;
  } else if (alpha_servo_degrees <= 0) {
    bar_size = fabs(bar_size);
    beta_servo_degrees += vertical_bar_size;
  }
  if (beta_servo_degrees >= vertical_degrees_scanned) {
    vertical_bar_size = -fabs(vertical_bar_size);
    if (seeking_up){
      printf("\n");
      seeking_up = !seeking_up;
    }
  } else if (beta_servo_degrees <= 0) {
    if (!seeking_up){
      printf("\n");
      seeking_up = !seeking_up;
    }
    vertical_bar_size = fabs(vertical_bar_size);
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
  float verticalAngleRad = beta_servo_degrees * (M_PI / 180);

  // Calculate the height (Z-coordinate) of the point
  float height = distance * tan(verticalAngleRad);

  return height;
}

int parseIntFromSerial() {
    char buffer[16]; // Adjust the buffer size as per your requirements
    int index = 0;

    // Read characters from serial until a non-numeric character is encountered
    while (1) {
        if (index < sizeof(buffer) - 1) { // Ensure buffer overflow protection
            int bytes_read = uart_read_bytes(0, (uint8_t *)&buffer[index], 1, portMAX_DELAY);
            if (bytes_read == 1) {
                if (isdigit((int)buffer[index])) {
                    index++;
                } else {
                    break; // Exit loop when a non-numeric character is encountered
                }
            }
        } else {
            // Handle buffer overflow
            break;
        }
    }

    // Null-terminate the string
    buffer[index] = '\0';

    // Convert the string to an integer
    int parsed_int = atoi(buffer);

    return parsed_int;
}

uint8_t readByteFromSerial() {
    uint8_t byte;

    // Read a single byte from the serial buffer
    int bytes_read = uart_read_bytes(0, &byte, 1, portMAX_DELAY);

    if (bytes_read == 1) {
        return byte; // Return the read byte
    } else {
        return 0; // Return 0 if no byte was read or an error occurred
    }
}


void targetAcquisition(){
  int high_target_edge = 0, low_target_edge = 0, left_target_edge = left_acq_edge, right_target_edge = 0;
  for (int y_degrees = low_acq_edge; y_degrees <= high_acq_edge; y_degrees += bar_size){
    for (int x_degrees = left_acq_edge; x_degrees <= right_acq_edge; x_degrees += bar_size){
      set_servos_degrees(x_degrees, y_degrees);
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
          first_target_coordinates[0] = lidar_distance_to_target * cos(x_degrees * (M_PI / 180)) * sin(y_degrees * (M_PI / 180));
          first_target_coordinates[1] = calculateHeight(lidar_distance_to_target);
          first_target_coordinates[2] = lidar_distance_to_target * sin(x_degrees * (M_PI / 180)) * sin(y_degrees * (M_PI / 180));
        }
        else{
          first_scan = !first_scan;
          second_target_coordinates[0] = lidar_distance_to_target * cos(x_degrees * (M_PI / 180)) * sin(y_degrees * (M_PI / 180));
          second_target_coordinates[1] = calculateHeight(lidar_distance_to_target);
          second_target_coordinates[2] = lidar_distance_to_target * sin(x_degrees * (M_PI / 180)) * sin(y_degrees * (M_PI / 180));
        }
      }
      
      old_target_speed = target_speed;
      target_speed = sqrt(pow(second_target_coordinates[0] - first_target_coordinates[0], 2) + pow(second_target_coordinates[1] - first_target_coordinates[1], 2) + pow(second_target_coordinates[2] - first_target_coordinates[2], 2)) / cycle_duration_ms;
      
      acceleration = target_speed - old_target_speed;
      
      heading = atan((second_target_coordinates[2] - first_target_coordinates[2]) / (second_target_coordinates[0] - first_target_coordinates[0]));
      if ((second_target_coordinates[2] - first_target_coordinates[2]) < 0 && (second_target_coordinates[0] - first_target_coordinates[0]) >= 0) {
        heading += M_PI;
      } else if ((second_target_coordinates[2] - first_target_coordinates[2]) < 0 && (second_target_coordinates[0] - first_target_coordinates[0]) < 0) {
        heading -= M_PI;
      }
      
      printf("Speed: %f Height: %f Acceleration: %f Heading: %f\n", target_speed, calculateHeight(lidar_distance_to_target), acceleration, heading * (180 / M_PI));
    }

    high_acq_edge = high_target_edge + bar_size;
    low_acq_edge = low_target_edge - bar_size;
    left_acq_edge = left_target_edge - bar_size;
    right_acq_edge = right_target_edge + bar_size;
  }
}


void app_main() {
  printf("Initializing...\n");
  init_pwm();
  gpio_set_direction(SERVO_PIN_ALPHA, GPIO_MODE_OUTPUT);
  gpio_set_direction(SERVO_PIN_BETA, GPIO_MODE_OUTPUT);

  if (!myLIDAR.begin()) {
    printf("Device did not acknowledge! Freezing.\n");
    while(1);
  }
  printf("LIDAR acknowledged!\n");

  // Initial scan for ground clutter filtration
  while (beta_servo_degrees <= vertical_degrees_scanned || alpha_servo_degrees <= horizontal_degrees_scanned){
    moveServos();
    ground_clutter[alpha_servo_bars][beta_servo_bars] = lidarMeasurement();
    vTaskDelay(cycle_duration_ms / portTICK_PERIOD_MS);
  }
  while(1) {
    // Wait until it's time to start the cycle
    while (esp_timer_get_time() / 1000 <= calculateNextCycleMillis()) {}
        
    if (uart_get_buffered_data_len(0, &buffered_data_length) >= 5) {
        if (!acquiring){
        ttl = ttl_scans;
        acquiring = !acquiring;
        low_acq_edge = parseIntFromSerial() - (acq_height/2);
        readByteFromSerial();
        left_acq_edge = parseIntFromSerial() - (acq_width/2);
        readByteFromSerial();
        right_acq_edge = left_acq_edge + acq_width;
        high_acq_edge = low_acq_edge + acq_height;
        } else {acquiring = !acquiring;}
    }

    if (ttl > 0 && acquiring){
        ttl--;
        targetAcquisition();
    }else {
        moveServos();
        float lidar_distance_to_target;
        lidar_distance_to_target = lidarMeasurement();
        if(ground_clutter[alpha_servo_bars][beta_servo_bars] <= lidar_distance_to_target + deviation){
        lidar_distance_to_target = max_lidar_range;
        }
        if (lidar_distance_to_target < max_lidar_range){
        printf("%f,", lidar_distance_to_target);
        }else {printf("0");}
    }

    current_cycle += 1;
    }
}