#pragma once

#define MOTOR_2250 1
#define MOTOR_3216 2


namespace motor_interface {

#if MOTOR_TYPE == MOTOR_2250

  // Encoder res: 4096
  // Gearhead gearing: 1:43

  // finger max movement: 95.48mm
  // motor max movement: 179263 incr.
  // pos_user = Feed/shaft * gearhead *  1/encoder_res * pos_internal
  // Feed/shaft = 95.48 / (1/43 * 1/4096 * 179263) = 95.48/1.0178 = 93.8102 [mm/rev]

  // Velocity factor = 1/60
  // vel_user = Velocity_factor * Feed/shaft * gearhead *  1/encoder_res * vel_internal

  // pos_user = (95.48/179263)    * pos_internal
  // vel_user = (95.48/179263)/60 * vel_internal

    constexpr float HOMING_OFFSET = 0.0; // mm
    constexpr float HOMING_SPEED = 50;
    constexpr float HOMING_ACC = 0.1;
    constexpr uint16_t HOMING_TORQUE_LIMIT = 400;
    constexpr float MAX_PROFILE_VEL = 150;

    constexpr float RATED_TORQUE = 26.2; // mNm
    constexpr float RATED_CURRENT = 0.85; // A
    constexpr unsigned int GEARHEAD_RATIO = 43;
    // constexpr unsigned int GRIPPER_GEARING = 93810; // um/rev
    constexpr unsigned int GRIPPER_GEARING = 95700; // um/rev
    constexpr uint16_t PWM_MAX_SPEED = GEARHEAD_RATIO*60; // rev/min
    constexpr float TORQUE_FACTOR = 1.0/21.25; // N/<raw torque unit>  // NOTE: gear depended!
    // constexpr float TORQUE_FORCE_FACTOR = 1/((float)GRIPPER_GEARING/(2*M_PI)/1e6);

    constexpr float GRIPPER_STROKE = 109.9; // mm

#elif MOTOR_TYPE == MOTOR_3216

    constexpr float HOMING_OFFSET = 1.0; // mm  (due to belt streching)
    constexpr float HOMING_SPEED = 50; // mm/s
    constexpr float HOMING_ACC = 0.1; // mm/s^2
    constexpr uint16_t HOMING_TORQUE_LIMIT = 250;

    constexpr float MAX_PROFILE_VEL = 150;

    constexpr float RATED_TORQUE = 38.0; // mNm
    constexpr float RATED_CURRENT = 1.1; // A
    constexpr unsigned int GEARHEAD_RATIO = 49;
    constexpr unsigned int GRIPPER_GEARING = 71793; // um/rev

    constexpr float GRIPPER_STROKE = 112.5; // mm

#endif
} // motor

