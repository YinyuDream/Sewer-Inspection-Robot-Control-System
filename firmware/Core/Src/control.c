#include "control.h"
#include <stddef.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Parameters
#define DT 0.01f
#define L 0.6f          // Wheelbase for Stanley (tuned value)
#define W 0.68f         // Track width
#define WHEEL_RADIUS 0.16f
#define CAR_LENGTH 0.3f // Distance from center to front axle
#define CORNER_CENTER 4.0f
#define RADIUS 1.6f
#define K_STANLEY 10.0f
#define K_SOFT 0.3f
#define BATTERY_VOLTAGE 12.0f
#define DESIRED_MAX_SPEED 1.0f

// PID Parameters
#define KP 8.0f
#define KI 1.0f
#define KD 0.05f
#define KF 0.5f

// State variables
static float target_speed_linear = 0.0f;
static float voltage_error_sum[2] = {0.0f, 0.0f}; // 0: Left, 1: Right
static float voltage_last_error[2] = {0.0f, 0.0f};
static float last_yaw = 0.0f;
static int first_run = 1;

static float normalize_angle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

static void geometric_calculation(float x, float y, float yaw, float vel, float *e, float *theta_e) {
    float d_path = CORNER_CENTER + RADIUS;
    float psi_path = 0.0f;
    *e = 0.0f;

    // Region 1: Right-Top Corner
    if (x > CORNER_CENTER && y > CORNER_CENTER) {
        float xc = CORNER_CENTER;
        float yc = CORNER_CENTER;
        float dist = sqrtf((x - xc) * (x - xc) + (y - yc) * (y - yc));
        *e = dist - RADIUS;
        float angle_to_center = atan2f(y - yc, x - xc);
        psi_path = angle_to_center + M_PI / 2.0f;
    }
    // Region 2: Left-Top Corner
    else if (x < -CORNER_CENTER && y > CORNER_CENTER) {
        float xc = -CORNER_CENTER;
        float yc = CORNER_CENTER;
        float dist = sqrtf((x - xc) * (x - xc) + (y - yc) * (y - yc));
        *e = dist - RADIUS;
        float angle_to_center = atan2f(y - yc, x - xc);
        psi_path = angle_to_center + M_PI / 2.0f;
    }
    // Region 3: Left-Bottom Corner
    else if (x < -CORNER_CENTER && y < -CORNER_CENTER) {
        float xc = -CORNER_CENTER;
        float yc = -CORNER_CENTER;
        float dist = sqrtf((x - xc) * (x - xc) + (y - yc) * (y - yc));
        *e = dist - RADIUS;
        float angle_to_center = atan2f(y - yc, x - xc);
        psi_path = angle_to_center + M_PI / 2.0f;
    }
    // Region 4: Right-Bottom Corner
    else if (x > CORNER_CENTER && y < -CORNER_CENTER) {
        float xc = CORNER_CENTER;
        float yc = -CORNER_CENTER;
        float dist = sqrtf((x - xc) * (x - xc) + (y - yc) * (y - yc));
        *e = dist - RADIUS;
        float angle_to_center = atan2f(y - yc, x - xc);
        psi_path = angle_to_center + M_PI / 2.0f;
    }
    // Region 5: Top Straight Line (y > corner_center) -> should use logic from python
    // Python: elif y > corner_center: e = y - d_path; psi_path = math.pi
    else if (y > CORNER_CENTER) {
        *e = y - d_path;
        psi_path = (float)M_PI;
    }
    // Region 6: Bottom Straight Line
    else if (y < -CORNER_CENTER) {
        *e = -d_path - y;
        psi_path = 0.0f;
    }
    // Region 7: Left Straight Line
    else if (x < -CORNER_CENTER) {
        *e = -d_path - x;
        psi_path = -(float)M_PI / 2.0f;
    }
    // Region 8: Right Straight Line
    else if (x > CORNER_CENTER) {
        *e = x - d_path;
        psi_path = (float)M_PI / 2.0f;
    }

    *theta_e = normalize_angle(psi_path - yaw);
}

static float calculate_voltage_pid(float target_w, float current_w, int wheel_idx) {
    float error = target_w - current_w;
    
    voltage_error_sum[wheel_idx] += error * DT;
    
    // Anti-windup
    if (voltage_error_sum[wheel_idx] > 12.0f) voltage_error_sum[wheel_idx] = 12.0f;
    if (voltage_error_sum[wheel_idx] < -12.0f) voltage_error_sum[wheel_idx] = -12.0f;
    
    float d_error = (error - voltage_last_error[wheel_idx]) / DT;
    voltage_last_error[wheel_idx] = error;
    
    float voltage = (KP * error) + (KI * voltage_error_sum[wheel_idx]) + (KF * target_w);
    
    if (voltage > BATTERY_VOLTAGE) voltage = BATTERY_VOLTAGE;
    if (voltage < -BATTERY_VOLTAGE) voltage = -BATTERY_VOLTAGE;
    
    return voltage;
}

/**
  * @brief 运动控制算法核心处理接口
  * @note 这里剥离了 FreeRTOS 的调度逻辑，专门负责处理纯粹的运动控制算法
  * @param status 指向接收到的机器人状态数据数组的指针 [x, y, yaw, v]
  * @param PWM_Value 指向电机PWM控制值数组的指针 (int16_t)
  */
void motion_control_algorithm(float *status, int16_t *PWM_Value) {
    if (status == NULL || PWM_Value == NULL) return;

    // 1. Extract State
    float x = status[0];
    float y = status[1];
    float yaw = status[2];
    float v = status[3];
    
    if (first_run) {
        last_yaw = yaw;
        first_run = 0;
    }

    // 2. Calculate Front Axle Position
    float front_x = x + CAR_LENGTH * cosf(yaw); // Use cosf for float
    float front_y = y + CAR_LENGTH * sinf(yaw);

    // 3. Soft Start Logic
    if (target_speed_linear < DESIRED_MAX_SPEED) {
        target_speed_linear += 0.5f * DT;
        if (target_speed_linear > DESIRED_MAX_SPEED) target_speed_linear = DESIRED_MAX_SPEED;
    }

    // 4. Stanley Control
    float e, theta_e;
    geometric_calculation(front_x, front_y, yaw, v, &e, &theta_e);
    
    // Calculate steering angle delta
    // delta = theta_e + atan2(k*e, v + k_soft)
    // Note: Python code has roll compensation, ignored here as no roll in status.
    float delta = theta_e + atan2f(K_STANLEY * e, v + K_SOFT);
    
    // Limit delta to +/- 45 degrees
    float limit_rad = 45.0f * (float)M_PI / 180.0f;
    if (delta > limit_rad) delta = limit_rad;
    if (delta < -limit_rad) delta = -limit_rad;

    // 5. Calculate Wheel Speeds (Differential Drive Kinematics with Ackermann equivalent)
    // omega = v * tan(delta) / L
    float omega = target_speed_linear * tanf(delta) / L;
    
    float v_left = target_speed_linear - (omega * W / 2.0f);
    float v_right = target_speed_linear + (omega * W / 2.0f);
    
    float w_target_l = v_left / WHEEL_RADIUS;
    float w_target_r = v_right / WHEEL_RADIUS;

    // 6. Estimate Current Wheel Speeds (Status only has v, yaw)
    // Estimate yaw rate
    float yaw_rate = normalize_angle(yaw - last_yaw) / DT;
    last_yaw = yaw;
    
    // w = (v +/- omega*W/2) / r
    // Use actual v and estimated yaw_rate
    float w_current_l = (v - yaw_rate * W / 2.0f) / WHEEL_RADIUS;
    float w_current_r = (v + yaw_rate * W / 2.0f) / WHEEL_RADIUS;

    // 7. Calculate Voltage (PID)
    float voltage_l = calculate_voltage_pid(w_target_l, w_current_l, 0);
    float voltage_r = calculate_voltage_pid(w_target_r, w_current_r, 1);

    // 8. Output to PWM_Value (Voltage * 1000)
    // Use int16_t for signed voltage values
    PWM_Value[0] = (int16_t)(voltage_l * 1000.0f);
    PWM_Value[1] = (int16_t)(voltage_r * 1000.0f);
    PWM_Value[2] = 0;
    PWM_Value[3] = 0;
}
