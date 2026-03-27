#include "control.h"
#include "tim.h"
#include <stddef.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Parameters
#define DT 0.01f
#define L 0.6f          // Wheelbase for Stanley
#define W 0.68f         // Track width
#define WHEEL_RADIUS 0.16f
#define CAR_LENGTH 0.3f // Distance from center to front axle
#define CORNER_CENTER 8.0f // Updated to match Python 
#define RADIUS 1.6f
#define K_STANLEY 4.0f    // Updated to match Python (was 10.0)
#define K_SOFT 2.0f       // Updated to match Python (was 0.3)
#define BATTERY_VOLTAGE 12.0f
#define DESIRED_MAX_SPEED 1.0f
#define SAFE_CORNER_SPEED_FACTOR 2.0f
#define DELTA_SLEW_RATE 100.0f
#define FILTER_ALPHA 0.7f

// PID Parameters
#define KP 5.0f // Updated to match Python (was 8.0)
#define KI 1.0f
#define KD 0.05f
#define KF 0.5f

// Roll Stabilization
#define K_ROLL_P 4.0f
#define K_ROLL_D 1.5f
#define K_ROLL_I 0.5f

// State variables
static float target_speed_linear = 0.0f;
static float voltage_error_sum[2] = {0.0f, 0.0f}; // 0: Left, 1: Right
static float voltage_last_error[2] = {0.0f, 0.0f};
static float error_roll_sum = 0.0f;

// Steering smoothing states
static float current_delta = 0.0f;
static float filtered_delta = 0.0f;

// Helper: Normalize angle to [-pi, pi]
static float normalize_angle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

// Helper: Calculate Roll and Yaw from Quaternion
// q: [x, y, z, w]
static void euler_from_quaternion(float x, float y, float z, float w, float *roll, float *yaw) {
    // Roll (x-axis rotation)
    float t0 = +2.0f * (w * x + y * z);
    float t1 = +1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(t0, t1);

    // Yaw (z-axis rotation)
    float t3 = +2.0f * (w * z + x * y);
    float t4 = +1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(t3, t4);
}

// Geometric Calculation (Stanley)
static void geometric_calculation(float x, float y, float yaw, float vel, float *e, float *theta_e) {
    float d_path = CORNER_CENTER + RADIUS;
    float psi_path = 0.0f;
    *e = 0.0f;

    // Logic aligned with VehicleDynamics.geometric_calculation in Python
    
    // 1. Right-Top Corner
    if (x > CORNER_CENTER && y > CORNER_CENTER) {
        float xc = CORNER_CENTER;
        float yc = CORNER_CENTER;
        float dist = sqrtf((x - xc) * (x - xc) + (y - yc) * (y - yc));
        *e = dist - RADIUS;
        float angle_to_center = atan2f(y - yc, x - xc);
        psi_path = angle_to_center + M_PI / 2.0f;
    }
    // 2. Left-Top Corner
    else if (x < -CORNER_CENTER && y > CORNER_CENTER) {
        float xc = -CORNER_CENTER;
        float yc = CORNER_CENTER;
        float dist = sqrtf((x - xc) * (x - xc) + (y - yc) * (y - yc));
        *e = dist - RADIUS;
        float angle_to_center = atan2f(y - yc, x - xc);
        psi_path = angle_to_center + M_PI / 2.0f;
    }
    // 3. Left-Bottom Corner
    else if (x < -CORNER_CENTER && y < -CORNER_CENTER) {
        float xc = -CORNER_CENTER;
        float yc = -CORNER_CENTER;
        float dist = sqrtf((x - xc) * (x - xc) + (y - yc) * (y - yc));
        *e = dist - RADIUS;
        float angle_to_center = atan2f(y - yc, x - xc);
        psi_path = angle_to_center + M_PI / 2.0f;
    }
    // 4. Right-Bottom Corner
    else if (x > CORNER_CENTER && y < -CORNER_CENTER) {
        float xc = CORNER_CENTER;
        float yc = -CORNER_CENTER;
        float dist = sqrtf((x - xc) * (x - xc) + (y - yc) * (y - yc));
        *e = dist - RADIUS;
        float angle_to_center = atan2f(y - yc, x - xc);
        psi_path = angle_to_center + M_PI / 2.0f;
    }
    // 5. Top Straight Line
    else if (y > CORNER_CENTER) {
        *e = y - d_path;
        psi_path = M_PI;
    }
    // 6. Bottom Straight Line
    else if (y < -CORNER_CENTER) {
        *e = -d_path - y;
        psi_path = 0.0f;
    }
    // 7. Left Straight Line
    else if (x < -CORNER_CENTER) {
        *e = -d_path - x;
        psi_path = -M_PI / 2.0f;
    }
    // 8. Right Straight Line
    else if (x > CORNER_CENTER) {
        *e = x - d_path;
        psi_path = M_PI / 2.0f;
    }

    *theta_e = normalize_angle(psi_path - yaw);
}

// PID Controller for Motor Voltage
static float calculate_voltage_pid(float target_w, float current_w, int wheel_idx) {
    return target_w;
    float error = target_w - current_w;
    
    voltage_error_sum[wheel_idx] += error * DT;
    
    // Anti-windup (Clamping sum to +/- 12.0)
    if (voltage_error_sum[wheel_idx] > 12.0f) voltage_error_sum[wheel_idx] = 12.0f;
    if (voltage_error_sum[wheel_idx] < -12.0f) voltage_error_sum[wheel_idx] = -12.0f;
    
    float d_error = (error - voltage_last_error[wheel_idx]) / DT;
    voltage_last_error[wheel_idx] = error;
    
    float voltage = (KP * error) + (KI * voltage_error_sum[wheel_idx]) + (KF * target_w) + (KD * d_error);
    
    // Battery Voltage Clamp
    if (voltage > BATTERY_VOLTAGE) voltage = BATTERY_VOLTAGE;
    if (voltage < -BATTERY_VOLTAGE) voltage = -BATTERY_VOLTAGE;
    
    return voltage;
}

static float clamp(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
  * @brief 运动控制算法核心处理接口
  * @param status 指向接收到的机器人状态数据数组的指针 [x, y, qx, qy, qz, qw, v, roll_rate, w_act_l, w_act_r]
  * @param PWM_Value 指向电机PWM/指令控制值数组的指针 [volt_l_mv, volt_r_mv, steer_l_mrad, steer_r_mrad]
  */
void motion_control_algorithm(float *status, uint16_t *PWM_Value) {
    if (status == NULL || PWM_Value == NULL) return;

    // 1. Unpack State
    float x_pos = status[0];
    float y_pos = status[1];
    
    // Quaternion Unpacking
    float qx = status[3];
    float qy = status[4];
    float qz = status[5];
    float qw = status[6];

    float vx = status[7];
    float vy = status[8];
    float vz = status[9];
    float v_linear = sqrtf(vx * vx + vy * vy + vz * vz);

    float roll_rate = status[10]; // Angular Velocity X
    float w_act_l = status[14];   // Actual Wheel Speed L (rad/s) - from 0x107
    float w_act_r = status[15];   // Actual Wheel Speed R (rad/s) - from 0x107

    // Calculate Yaw and Roll from Quaternion
    float roll = 0.0f;
    float yaw = 0.0f;
    euler_from_quaternion(qx, qy, qz, qw, &roll, &yaw);

    // 2. Corner Speed Adjustment Logic
    // expected_speed = desired_max / (1.0 + factor * abs(delta))
    float expected_speed = DESIRED_MAX_SPEED / (1.0f + SAFE_CORNER_SPEED_FACTOR * fabsf(current_delta));
    if (expected_speed < 0.4f) expected_speed = 0.4f;

    // Smooth Start / Speed Ramp
    if (target_speed_linear < expected_speed) {
        target_speed_linear += 0.5f * DT;
    } else {
        target_speed_linear -= 1.0f * DT;
    }
    target_speed_linear = clamp(target_speed_linear, 0.0f, 1.2f);

    // 3. Stanley Control
    // Calculate Front Axle Position
    float front_x = x_pos + CAR_LENGTH * cosf(yaw);
    float front_y = y_pos + CAR_LENGTH * sinf(yaw);
    
    float e_geo, theta_e;
    geometric_calculation(front_x, front_y, yaw, v_linear, &e_geo, &theta_e);

    // Update Roll Error Integral
    error_roll_sum += roll * DT;

    // Calculate Delta components
    float delta_geo = theta_e + atan2f(K_STANLEY * e_geo, v_linear + K_SOFT);
    float delta_roll = -K_ROLL_P * roll - K_ROLL_D * roll_rate - K_ROLL_I * error_roll_sum;
    
    float raw_target_delta = delta_geo + delta_roll;
    
    // Clamp Raw Delta (+/- 45 deg)
    float limit_45 = 45.0f * (M_PI / 180.0f);
    raw_target_delta = clamp(raw_target_delta, -limit_45, limit_45);

    // 4. Steering Smoothing (Low Pass + Slew Rate)
    // Low Pass Filter
    filtered_delta = filtered_delta * (1.0f - FILTER_ALPHA) + raw_target_delta * FILTER_ALPHA;
    
    // Slew Rate Limit
    float max_delta_change = DELTA_SLEW_RATE * DT;
    float delta_change = filtered_delta - current_delta;
    delta_change = clamp(delta_change, -max_delta_change, max_delta_change);
    
    current_delta += delta_change;
    float delta = current_delta;

    // 5. Calculate Ackermann Angles (Output for Steer)
    float delta_l = 0.0f;
    float delta_r = 0.0f;
    if (fabsf(delta) > 1e-6f) {
        float tan_delta = tanf(delta);
        delta_l = atanf(2.0f * L * tan_delta / (2.0f * L - W * tan_delta));
        delta_r = atanf(2.0f * L * tan_delta / (2.0f * L + W * tan_delta));
    }

    // 6. Calculate Wheel Target Speeds (Differential Drive Logic)
    // omega = v * tan(delta) / L
    float omega = target_speed_linear * tanf(delta) / L;
    
    float v_left = target_speed_linear - (omega * W / 2.0f);
    float v_right = target_speed_linear + (omega * W / 2.0f);
    
    float w_target_l = v_left / WHEEL_RADIUS;
    float w_target_r = v_right / WHEEL_RADIUS;

    // 7. Motor PID Control
    float voltage_l = calculate_voltage_pid(w_target_l, w_act_l, 0);
    float voltage_r = calculate_voltage_pid(w_target_r, w_act_r, 1);
    
    if(isnan(voltage_l)||isinf(voltage_l)) voltage_l = 0.0f;
    if(isnan(voltage_r)||isinf(voltage_r)) voltage_r = 0.0f;
    if(isnan(delta_l)||isinf(delta_l)) delta_l = 0.0f;
    if(isnan(delta_r)||isinf(delta_r)) delta_r = 0.0f;

    // 8. Output to PWM_Value
    // Map: [0]=>Volt_L_mV, [1]=>Volt_R_mV, [2]=>Steer_L_mrad, [3]=>Steer_R_mrad
    PWM_Value[0] = (uint16_t)(voltage_l * 1000.0f) + 0x8000;
    PWM_Value[1] = (uint16_t)(voltage_r * 1000.0f) + 0x8000;
    PWM_Value[2] = (uint16_t)(delta_l * 1000.0f) + 0x8000;
    PWM_Value[3] = (uint16_t)(delta_r * 1000.0f) + 0x8000;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)( ((uint32_t)PWM_Value[0] * 1000) / 4095 ));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)( ((uint32_t)PWM_Value[1] * 1000) / 4095 ));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint16_t)( ((uint32_t)PWM_Value[2] * 1000) / 4095 ));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t)( ((uint32_t)PWM_Value[3] * 1000) / 4095 ));
}
