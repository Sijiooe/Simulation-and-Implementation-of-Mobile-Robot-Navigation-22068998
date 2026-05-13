/*
 * ============================================================
 * ORIGINAL WEBOTS PIONEER 3-AT LIDAR OBSTACLE AVOIDANCE
 * + GPS + COMPASS ADDED
 * + NAVIGATION TO GOAL ZONE
 * ============================================================
 *
 * This keeps the ORIGINAL obstacle avoidance behaviour.
 *
 * Added features:
 * 1. GPS sensor
 * 2. Compass sensor
 * 3. Console prints robot position + heading
 * 4. Goal zone   [24.0 <= X <= 25.0] && [24.0 <= Y <= 25.6]
 *    - print "Goal Found"
 *    - stop 3 s
 *    - turn 180°
 *    - stop 1 s
 *    - drive forward (avoidance stays off in pause zone)
 * 5. Pause zone  [20 <= X <= 26] && [20 <= Y <= 26]
 *    - obstacle avoidance disabled, robot drives straight *towards goal*
 * 6. NAVIGATION TECHNIQUE:
 *    Reactive obstacle avoidance with proportional goal-seeking
 *    (a bug‑like algorithm).
 *
 *    Outside the pause zone:
 *      - If no obstacle is close, the robot steers directly toward
 *        the centre of the goal zone using a proportional heading
 *        controller.
 *      - If an obstacle is detected (lidar), the original Braitenberg
 *        avoidance takes over completely, turning the robot away
 *        from the obstacle.
 *    Inside the pause zone:
 *      - Obstacle avoidance is suspended; the robot only uses the
 *        proportional heading controller to drive to the goal.
 *
 *    This combination yields a simple “go‑to‑goal unless blocked,
 *    then avoid” behaviour, similar to a Bug‑0 / Bug‑1 navigation
 *    strategy.
 * ============================================================
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <webots/robot.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/compass.h>

#define TIME_STEP          32
#define MAX_SPEED          6.4
#define CRUISING_SPEED     5.0
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR    0.9
#define BACK_SLOWDOWN      0.9

// Goal zone (NEW narrower range)
#define GOAL_MIN     24.0
#define GOAL_MAX     25.6

// Pause zone (unchanged)
#define PAUSE_MIN    20.0
#define PAUSE_MAX    26.0

// Turning parameters
#define TURN_SPEED       2.0      // speed during the 180° turn
#define ANGLE_TOLERANCE  0.1      // rad (close enough to 180°)

// Navigation constants – centre of goal zone
#define GOAL_X        24.5        // (24.0+25.0)/2
#define GOAL_Y        24.5
#define KP_TURN       0.7         // proportional gain for heading control

// ------------------------------------------------
// Gaussian function
// ------------------------------------------------
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) *
         exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

// ============================================================
// MAIN
// ============================================================
int main(int argc, char **argv) {

  wb_robot_init();

  // ------------------------------------------------
  // Devices
  // ------------------------------------------------
  WbDeviceTag lms291 = wb_robot_get_device("Sick LMS 291");

  WbDeviceTag front_left_wheel  = wb_robot_get_device("front left wheel");
  WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
  WbDeviceTag back_left_wheel   = wb_robot_get_device("back left wheel");
  WbDeviceTag back_right_wheel  = wb_robot_get_device("back right wheel");

  // NEW DEVICES
  WbDeviceTag gps     = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");

  // ------------------------------------------------
  // Enable Sensors
  // ------------------------------------------------
  wb_lidar_enable(lms291, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  wb_compass_enable(compass, TIME_STEP);

  // ------------------------------------------------
  // Lidar Setup
  // ------------------------------------------------
  const int width = wb_lidar_get_horizontal_resolution(lms291);
  const int half_width = width / 2;

  const double max_range = wb_lidar_get_max_range(lms291);
  const double range_threshold = max_range / 20.0;

  const float *values = NULL;

  // ------------------------------------------------
  // Braitenberg Coefficients
  // ------------------------------------------------
  double *coeff = (double *)malloc(sizeof(double) * width);

  int i, j;
  for (i = 0; i < width; i++)
    coeff[i] = gaussian(i, half_width, width / 5.0);

  // ------------------------------------------------
  // Motors
  // ------------------------------------------------
  wb_motor_set_position(front_left_wheel, INFINITY);
  wb_motor_set_position(front_right_wheel, INFINITY);
  wb_motor_set_position(back_left_wheel, INFINITY);
  wb_motor_set_position(back_right_wheel, INFINITY);

  double front_left_speed  = 0.0;
  double front_right_speed = 0.0;
  double back_left_speed   = 0.0;
  double back_right_speed  = 0.0;

  wb_motor_set_velocity(front_left_wheel, front_left_speed);
  wb_motor_set_velocity(front_right_wheel, front_right_speed);
  wb_motor_set_velocity(back_left_wheel, back_left_speed);
  wb_motor_set_velocity(back_right_wheel, back_right_speed);

  // ------------------------------------------------
  // Dynamic Variables
  // ------------------------------------------------
  double left_obstacle  = 0.0;
  double right_obstacle = 0.0;

  // Goal sequence state machine
  static bool   goal_sequence_active = false;
  static bool   goal_triggered       = false;
  static int    goal_state           = 0;   // 0=wait3, 1=turn, 2=wait1
  static int    goal_counter         = 0;
  static double turn_start_heading   = 0.0;

  const int wait3_steps = (int)(3.0 * 1000 / TIME_STEP);   // 3 seconds
  const int wait1_steps = (int)(1.0 * 1000 / TIME_STEP);   // 1 second

  printf("Original obstacle avoidance + GPS + Compass + Navigation started.\n");
  printf("Goal zone: [%.1f, %.1f] x [%.1f, %.1f]\n", GOAL_MIN, GOAL_MAX, GOAL_MIN, GOAL_MAX);
  printf("Pause zone: [%.1f, %.1f] x [%.1f, %.1f]\n", PAUSE_MIN, PAUSE_MAX, PAUSE_MIN, PAUSE_MAX);

  // ============================================================
  // MAIN LOOP
  // ============================================================
  while (wb_robot_step(TIME_STEP) != -1) {

    // ------------------------------------------------
    // Read GPS
    // ------------------------------------------------
    const double *pos = wb_gps_get_values(gps);
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];

    // ------------------------------------------------
    // Read Compass
    // ------------------------------------------------
    const double *north = wb_compass_get_values(compass);
    double heading = atan2(north[0], north[2]);

    // ------------------------------------------------
    // Determine zones
    // ------------------------------------------------
    bool inside_goal  = (x >= GOAL_MIN && x <= GOAL_MAX && y >= GOAL_MIN && y <= GOAL_MAX);
    bool inside_pause = (x >= PAUSE_MIN && x <= PAUSE_MAX && y >= PAUSE_MIN && y <= PAUSE_MAX);

    // ------------------------------------------------
    // Goal sequence state machine
    // ------------------------------------------------
    if (!goal_sequence_active) {
      // ---- Normal operation (not in a goal sequence) ----
      if (inside_goal && !goal_triggered) {
        // Enter goal sequence
        goal_sequence_active = true;
        goal_triggered       = true;
        goal_state           = 0;
        goal_counter         = 0;
        // Stop immediately
        front_left_speed  = 0.0;
        front_right_speed = 0.0;
        back_left_speed   = 0.0;
        back_right_speed  = 0.0;
        printf("Goal Found\n");
      } else {
        // ----------------------------------------------------
        // NAVIGATION LOGIC: proportional heading to goal centre
        // ----------------------------------------------------
        double desired_heading = atan2(GOAL_Y - y, GOAL_X - x);
        double heading_error = desired_heading - heading;

        // Normalise error to [-PI, PI]
        while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

        // Proportional turn amount
        double turn_cmd = KP_TURN * heading_error;
        // Clamp to MAX_SPEED to avoid saturating too much
        if (turn_cmd >  MAX_SPEED) turn_cmd =  MAX_SPEED;
        if (turn_cmd < -MAX_SPEED) turn_cmd = -MAX_SPEED;

        // Obstacle avoidance is active only outside the pause zone
        bool use_avoidance = !inside_pause;
        if (use_avoidance) {
          // Read lidar and compute obstacle levels
          values = wb_lidar_get_range_image(lms291);
          left_obstacle  = 0.0;
          right_obstacle = 0.0;

          for (i = 0; i < half_width; i++) {
            // Left side
            if (values[i] < range_threshold)
              left_obstacle += coeff[i] * (1.0 - values[i] / max_range);
            // Right side
            j = width - i - 1;
            if (values[j] < range_threshold)
              right_obstacle += coeff[i] * (1.0 - values[j] / max_range);
          }

          double obstacle = left_obstacle + right_obstacle;

          if (obstacle > OBSTACLE_THRESHOLD) {
            // Obstacle present -> pure avoidance (ignore goal seeking)
            double speed_factor =
                (1.0 - DECREASE_FACTOR * obstacle) * MAX_SPEED / obstacle;
            front_left_speed  = speed_factor * left_obstacle;
            front_right_speed = speed_factor * right_obstacle;
            back_left_speed   = BACK_SLOWDOWN * front_left_speed;
            back_right_speed  = BACK_SLOWDOWN * front_right_speed;
          } else {
            // No obstacle nearby -> go‑to‑goal
            front_left_speed  = CRUISING_SPEED - turn_cmd;
            front_right_speed = CRUISING_SPEED + turn_cmd;
            back_left_speed   = BACK_SLOWDOWN * front_left_speed;
            back_right_speed  = BACK_SLOWDOWN * front_right_speed;
          }
        } else {
          // Inside pause zone -> pure go‑to‑goal (no lidar)
          front_left_speed  = CRUISING_SPEED - turn_cmd;
          front_right_speed = CRUISING_SPEED + turn_cmd;
          back_left_speed   = BACK_SLOWDOWN * front_left_speed;
          back_right_speed  = BACK_SLOWDOWN * front_right_speed;
        }

        // Reset goal trigger flag when robot leaves the goal zone
        if (!inside_goal)
          goal_triggered = false;
      }

    } else {
      // ---- Goal sequence is running (stop, turn 180°, stop) ----
      switch (goal_state) {
        case 0: // ⏳ Wait 3 seconds
          front_left_speed  = 0.0;
          front_right_speed = 0.0;
          back_left_speed   = 0.0;
          back_right_speed  = 0.0;
          goal_counter++;
          if (goal_counter >= wait3_steps) {
            goal_state = 1;
            goal_counter = 0;
            turn_start_heading = heading;
          }
          break;

        case 1: // Turn 180 degrees in place
          front_left_speed  =  TURN_SPEED;
          back_left_speed   =  TURN_SPEED;
          front_right_speed = -TURN_SPEED;
          back_right_speed  = -TURN_SPEED;

          {
            double diff = heading - turn_start_heading;
            // Normalise to [-PI, PI]
            diff = fmod(diff + M_PI, 2.0 * M_PI);
            if (diff < 0.0) diff += 2.0 * M_PI;
            diff -= M_PI;

            if (fabs(diff) >= M_PI - ANGLE_TOLERANCE) {
              // Turn complete
              front_left_speed  = 0.0;
              front_right_speed = 0.0;
              back_left_speed   = 0.0;
              back_right_speed  = 0.0;
              goal_state = 2;
              goal_counter = 0;
            }
          }
          break;

        case 2: // Wait 1 second
          front_left_speed  = 0.0;
          front_right_speed = 0.0;
          back_left_speed   = 0.0;
          back_right_speed  = 0.0;
          goal_counter++;
          if (goal_counter >= wait1_steps) {
            // Goal sequence finished
            goal_sequence_active = false;
            // goal_triggered stays true until robot leaves goal zone
          }
          break;
      }
    }

    // ------------------------------------------------
    // Apply Motor Speeds
    // ------------------------------------------------
    wb_motor_set_velocity(front_left_wheel, front_left_speed);
    wb_motor_set_velocity(front_right_wheel, front_right_speed);
    wb_motor_set_velocity(back_left_wheel, back_left_speed);
    wb_motor_set_velocity(back_right_wheel, back_right_speed);

    // ------------------------------------------------
    // Print Position + Heading
    // ------------------------------------------------
    printf("GPS: X=%.2f Y=%.2f Z=%.2f | Heading=%.2f rad\n", x, y, z, heading);

    // Reset per‑loop obstacle accumulators
    left_obstacle  = 0.0;
    right_obstacle = 0.0;
  }

  free(coeff);
  wb_robot_cleanup();
  return 0;
}
