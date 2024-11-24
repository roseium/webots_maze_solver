// HEADER FILES

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/gps.h>


// CONSTANTS

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define WALL_DISTANCE_THRESHOLD 110.0
#define LIGHT_INTENSITY_THRESHOLD 500.0
#define GPS_POSITION_THRESHOLD 0.05
#define NUM_PROXIMITY_SENSORS 8


// STRUCT TO STORE COORDINATES AND LIGHT INTENSITY

typedef struct 
{
    double light_intensity;
    double position_x;
    double position_y;
    double position_z;
} DeadEnd;

// PROTOTYPING FUNCTIONS

void deadend_store(DeadEnd *dead_end_records, int *dead_end_count, WbDeviceTag light_sensor, const double *gps_position);
void following_wall(const WbDeviceTag *proximity_sensors, double *motor_speed_left, double *motor_speed_right);
void device_initialization(WbDeviceTag *motor_left, WbDeviceTag *motor_right, WbDeviceTag *proximity_sensors, WbDeviceTag *light_sensor, WbDeviceTag *gps_device);

bool deadend_detect(const WbDeviceTag *proximity_sensors);
bool check_target_reached(const double *gps_position, const DeadEnd *target);

double constrain_value(double value, double min_value, double max_value);

DeadEnd find_highest_intensity_dead_end(const DeadEnd *dead_end_records, int record_count);


// MAIN FUNCTION

int main(int argc, char **argv)
{
    wb_robot_init();   // Initializing the robot
    
    // Initializing robot motors and sensors
    
    WbDeviceTag motor_left, motor_right;
    WbDeviceTag proximity_sensors[NUM_PROXIMITY_SENSORS];
    WbDeviceTag light_sensor, gps_device;
    device_initialization(&motor_left, &motor_right, proximity_sensors, &light_sensor, &gps_device);
    
    // Resettings values

    DeadEnd dead_end_records[10] = {0};
    int dead_end_count = 0;
    DeadEnd brightest_dead_end = {0};
    double motor_speed_left = MAX_SPEED, motor_speed_right = MAX_SPEED;
    
    
    // while loop to keep the robot running
    
    while (wb_robot_step(TIME_STEP) != -1)
    {
        const double *gps_position = wb_gps_get_values(gps_device);

        if (deadend_detect(proximity_sensors))
        {
            deadend_store(dead_end_records, &dead_end_count, light_sensor, gps_position);

            if (dead_end_count == 10)
            {
                brightest_dead_end = find_highest_intensity_dead_end(dead_end_records, 10);
                printf("\nBrightest dead end: Light intensity %.2f at (%.2f, %.2f, %.2f)\n",brightest_dead_end.light_intensity, brightest_dead_end.position_x, brightest_dead_end.position_y, brightest_dead_end.position_z);
            }
        }
        else
        {
            following_wall(proximity_sensors, &motor_speed_left, &motor_speed_right);
        }

        wb_motor_set_velocity(motor_left, constrain_value(motor_speed_left, -MAX_SPEED, MAX_SPEED));
        wb_motor_set_velocity(motor_right, constrain_value(motor_speed_right, -MAX_SPEED, MAX_SPEED));
        
        
        if (check_target_reached(gps_position, &brightest_dead_end))
        {
            printf("\nTarget reached! Stopping robot.\n");
            wb_motor_set_velocity(motor_left, 0);
            wb_motor_set_velocity(motor_right, 0);
            break;
        }
    }

    wb_robot_cleanup();
    
    return 0;
}


// This function initializes all the devices used in the e-puck

void device_initialization(WbDeviceTag *motor_left, WbDeviceTag *motor_right, WbDeviceTag *proximity_sensors, WbDeviceTag *light_sensor, WbDeviceTag *gps_device)
{
    *motor_left = wb_robot_get_device("left wheel motor");
    *motor_right = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(*motor_left, INFINITY);
    wb_motor_set_position(*motor_right, INFINITY);

    for (int i = 0; i < NUM_PROXIMITY_SENSORS; i++)
    {
        char sensor_name[8];
        snprintf(sensor_name, sizeof(sensor_name), "ps%d", i);
        proximity_sensors[i] = wb_robot_get_device(sensor_name);

        wb_distance_sensor_enable(proximity_sensors[i], TIME_STEP);
    }

    *light_sensor = wb_robot_get_device("ls0");
    wb_light_sensor_enable(*light_sensor, TIME_STEP);

    *gps_device = wb_robot_get_device("gps");
    wb_gps_enable(*gps_device, TIME_STEP);
}


// Function that detects dead ends

bool deadend_detect(const WbDeviceTag *proximity_sensors)
{
    static int deadend_conseccount = 0;
    static double last_detection_time = 0;

    double current_time = wb_robot_get_time();
    double front_sensor_reading = wb_distance_sensor_get_value(proximity_sensors[0]);

    if (front_sensor_reading > 100 && 
        (deadend_conseccount == 0 || (current_time - last_detection_time) > 1.7))
    {
        last_detection_time = current_time;
        deadend_conseccount++;
    }

    if (deadend_conseccount >= 2)
    {
        deadend_conseccount = 0;
        return true;
    }

    if ((current_time - last_detection_time) > 10.0)
    {
        deadend_conseccount = 0;
    }
    return false;
}


// Function that stores all the dead end coordinates

void deadend_store(DeadEnd *dead_end_records, int *dead_end_count, 
                  WbDeviceTag light_sensor, const double *gps_position) {
    if (*dead_end_count < 10) {DeadEnd *current_dead_end = &dead_end_records[*dead_end_count]; current_dead_end->light_intensity = wb_light_sensor_get_value(light_sensor); current_dead_end->position_x = gps_position[0]; current_dead_end->position_y = gps_position[1]; current_dead_end->position_z = gps_position[2];

        printf("\nDead end %d: Light intensity %.2f at (%.2f, %.2f, %.2f)\n", *dead_end_count + 1, current_dead_end->light_intensity,current_dead_end->position_x, current_dead_end->position_y, current_dead_end->position_z);
        
        (*dead_end_count)++;
    }
}



// Function that enables the robot to navigate through the maze

void following_wall(const WbDeviceTag *proximity_sensors, double *motor_speed_left, double *motor_speed_right)
{
    bool wall_on_left = wb_distance_sensor_get_value(proximity_sensors[5]) > WALL_DISTANCE_THRESHOLD;
    bool approaching_corner = wb_distance_sensor_get_value(proximity_sensors[6]) > WALL_DISTANCE_THRESHOLD;
    bool wall_in_front = wb_distance_sensor_get_value(proximity_sensors[7]) > WALL_DISTANCE_THRESHOLD;

    if (wall_in_front) {
        *motor_speed_left = MAX_SPEED;
        *motor_speed_right = -MAX_SPEED;
    }
    else if (wall_on_left)
    {
        *motor_speed_left = *motor_speed_right = MAX_SPEED / 2;
    }
    else if (approaching_corner)
    {
        *motor_speed_left = MAX_SPEED;
        *motor_speed_right = MAX_SPEED / 4;
    }
    else
    {
        *motor_speed_left = MAX_SPEED / 4;
        *motor_speed_right = MAX_SPEED;
    }
}




// Checks if it has reached the target position

bool check_target_reached(const double *gps_position, const DeadEnd *target)
{
    return (fabs(gps_position[0] - target->position_x) < GPS_POSITION_THRESHOLD && fabs(gps_position[1] - target->position_y) < GPS_POSITION_THRESHOLD && fabs(gps_position[2] - target->position_z) < GPS_POSITION_THRESHOLD);
}


// constrains

double constrain_value(double value, double min_value, double max_value)
{
    return fmin(fmax(value, min_value), max_value);
}


// checks for the coordinates tied to the highest intensity

DeadEnd find_highest_intensity_dead_end(const DeadEnd *dead_end_records, int record_count)
{
    DeadEnd brightest_dead_end = dead_end_records[0];

    for (int i = 1; i < record_count; i++)
    {
        if (dead_end_records[i].light_intensity > brightest_dead_end.light_intensity)
        {
            brightest_dead_end = dead_end_records[i];
        }
    }
    
    return brightest_dead_end;
}
