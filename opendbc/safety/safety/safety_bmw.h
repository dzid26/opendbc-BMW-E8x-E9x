#pragma once

#include "safety_declarations.h"
static float interpolate(struct lookup_t xy, float x);

#define BMW_LIMITS(steer, rate_up, rate_down) { \
  .max_torque = (steer), \
  .max_rate_up = (rate_up), \
  .max_rate_down = (rate_down), \
  .max_rt_delta = 112, \
  .driver_torque_allowance = 50, \
  .driver_torque_multiplier = 2, \
  .type = TorqueDriverLimited, \
   /* the EPS faults when the steering angle is above a certain threshold for too long. to prevent this, */ \
   /* we allow setting CF_Lkas_ActToi bit to 0 while maintaining the requested torque value for two consecutive frames */ \
  .min_valid_request_frames = 89, \
  .max_invalid_request_frames = 2, \
  .min_valid_request_rt_interval = 810000,  /* 810ms; a ~10% buffer on cutting every 90 frames */ \
  .has_steer_req_tolerance = true, \
}

// CAN msgs we care about
#define BMW_EngineAndBrake 0xA8
#define BMW_AccPedal 0xAA
#define BMW_Speed 0x1A0
#define BMW_SteeringWheelAngle_slow 0xC8
#define BMW_CruiseControlStatus 0x200
#define BMW_DynamicCruiseControlStatus 0x193
#define BMW_CruiseControlStalk 0x194
#define BMW_TransmissionDataDisplay 0x1D2

#define BMW_PT_CAN 0
#define BMW_F_CAN 1
#define BMW_AUX_CAN 2


RxCheck bmw_rx_checks[] = {  // todo add .check_checksum
  {.msg = {{BMW_EngineAndBrake,       BMW_PT_CAN, 8, .max_counter = 14U, .frequency = 100U, .ignore_checksum = true}, { 0 }, { 0 }}},
  {.msg = {{BMW_AccPedal,             BMW_PT_CAN, 8, .max_counter = 14U, .frequency = 100U, .ignore_checksum = true}, { 0 }, { 0 }}},
  {.msg = {{BMW_Speed,                BMW_PT_CAN, 8, .max_counter = 14U, .frequency = 50U, .ignore_checksum = true}, { 0 }, { 0 }}},
  // {.msg = {{BMW_SteeringWheelAngle_slow,   BMW_PT_CAN, 6, .ignore_counter = true, .frequency = 5U, .ignore_checksum = true}, { 0 }, { 0 }}}, // todo if uesed, maybe add to bmw_get_counter
  {.msg = {{BMW_TransmissionDataDisplay,    BMW_PT_CAN, 6, .max_counter = 14U, .frequency = 5U, .ignore_checksum = true}, { 0 }, { 0 }}},
  {.msg = {{BMW_DynamicCruiseControlStatus, BMW_PT_CAN, 8, .max_counter = 14U, .frequency = 5U, .ignore_checksum = true},
           {BMW_CruiseControlStatus,  BMW_PT_CAN, 8, .ignore_counter = true, .frequency = 5U, .ignore_checksum = true}, { 0 }}},
  // {.msg = {{BMW_SteeringWheelAngle_slow,   BMW_PT_CAN, 6, .max_counter = 0U, .frequency = 5U, .ignore_checksum = true}, { 0 }, { 0 }}},
  // todo cruise control type dependant, use param:
  // {.msg = {{0x22f,  BMW_F_CAN, 8, .frequency = 100U}, { 0 }, { 0 }}},
  // {.msg = {{0x22f,  BMW_AUX_CAN, 8, .frequency = 100U}, { 0 }, { 0 }}},
};


static uint8_t bmw_get_counter(const CANPacket_t *to_push) {
  uint8_t cnt = 0;
  int addr = GET_ADDR(to_push);
  if (addr == BMW_DynamicCruiseControlStatus) {
    cnt = (GET_BYTE(to_push, 0) >> 4) & 0xFU;
  } else if (addr == BMW_TransmissionDataDisplay) {
    cnt = (GET_BYTE(to_push, 3) >> 4) & 0xFU;
  } else if (addr == BMW_Speed) {
    cnt = (GET_BYTE(to_push, 6) >> 4) & 0xFU;
  } else {
    cnt = GET_BYTE(to_push, 1) & 0xFU;
  }
  return cnt;
}

const CanMsg BMW_TX_MSGS[] = {
  {BMW_CruiseControlStalk, BMW_PT_CAN, 4, false},   // Normal cruise control send status on PT-CAN
  {BMW_CruiseControlStalk, BMW_F_CAN, 4, false},    // Dynamic cruise control send status on F-CAN
  {0x22e, BMW_F_CAN, 5, false},    // STEPPER_SERVO_CAN is allowed on F-CAN network
  {0x22e, BMW_AUX_CAN, 5, false},  // or an standalone network
};

#define KPH_TO_MS 0.277778

#define CAN_BMW_SPEED_FAC 0.1
#define CAN_BMW_ANGLE_FAC 0.04395
#define CAN_BMW_ACC_FAC 0.025
#define CAN_ACTUATOR_POS_FAC 0.125
#define CAN_ACTUATOR_TQ_FAC 0.125
#define CAN_ACTUATOR_CONTROL_STATUS_SOFTOFF_BIT 2

bool bmw_fmax_limit_check(float val, const float MAX_VAL, const float MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

// rounding error margin
float BMW_MARGIN = 0.1;

// #define BMW_LAT_ACC_MAX 3.0 // EU guideline

// // steering angle based on EU 3m/s2 lat acc limit for 2.76m wheelbase and 16.0 steer ratio
const struct lookup_t BMW_LOOKUP_MAX_ANGLE = {
    {5., 15., 25.},     // m/s
    {303.6, 33.7, 12.1}};  // deg


const struct lookup_t BMW_ANGLE_RATE_WINDUP = { // deg/s windup rate limit
    {0., 5., 25.},      // m/s
    {500., 80., 40.}};  // deg/s

const struct lookup_t BMW_ANGLE_RATE_UNWIND = { // deg/s unwind rate limit
    {0., 5., 25.},      // m/s
    {500., 350., 50.}}; // deg/s

const struct lookup_t BMW_MAX_TQ_RATE = {
    {0., 5., 15.},      // m/s
    {16., 8., 1.}};   // Nm/10ms

// state of angle limits
float bmw_rt_angle_last = 0.; // last actual angle

float angle_rate_up = 0;
float angle_rate_down = 0;
float bmw_max_angle = 0;
float max_tq_rate = 0;

int lever_position = -1; //0 is when no ignition, so -1 unset
float bmw_speed = 0;
float actuator_torque = 0;


static void bmw_rx_hook(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  bool cruise_engaged = false;
  if ((addr == BMW_DynamicCruiseControlStatus) || (addr == BMW_CruiseControlStatus)) { //handles both vehicle options VO544 and Vo540
    if (addr == BMW_DynamicCruiseControlStatus) { //dynamic cruise control
      cruise_engaged = (((GET_BYTE(to_push, 5) >> 3) & 0x1U) == 1U);
    } else if (addr == BMW_CruiseControlStatus) { //normal cruise control option
      cruise_engaged = (((GET_BYTE(to_push, 1) >> 5) & 0x1U) == 1U);
    } else {
      cruise_engaged = false;
    }
    pcm_cruise_check(cruise_engaged);
  }

  if (addr == BMW_CruiseControlStalk){ //disable on cruise stalk cancel
    if ((GET_BYTE(to_push, 2) & 0x90) != 0x0){
      controls_allowed = false;
    }
  }
  if (addr == BMW_TransmissionDataDisplay) {
    lever_position = GET_BYTE(to_push, 0) & 0xF;
    if (lever_position != ((GET_BYTE(to_push, 0) >> 4) ^ 0xF)) { //check against shift lever compliment signal
      lever_position = -1; //invalid
    }
    // if not in Drive
    if (lever_position != 8 ){
      controls_allowed = false;
    }
  }

  //get vehicle speed
  if (addr == BMW_Speed) {
    bmw_speed = to_signed(((GET_BYTE(to_push, 1) & 0xF) << 8) + GET_BYTE(to_push, 0), 12) * CAN_BMW_SPEED_FAC * KPH_TO_MS; //raw to km/h to m/s
    angle_rate_up = interpolate(BMW_ANGLE_RATE_WINDUP, bmw_speed) + BMW_MARGIN;   // deg/1s
    angle_rate_down = interpolate(BMW_ANGLE_RATE_UNWIND, bmw_speed) + BMW_MARGIN; // deg/1s
    bmw_max_angle = interpolate(BMW_LOOKUP_MAX_ANGLE, bmw_speed) + BMW_MARGIN;
    max_tq_rate = interpolate(BMW_MAX_TQ_RATE, bmw_speed) + BMW_MARGIN;

    // check moving forward and reverse
    vehicle_moving = (GET_BYTE(to_push, 1) & 0x30U) != 0U;

    // // check lateral acceleration limits
    // float bmw_lat_acc = to_signed((GET_BYTE(to_push, 4) << 4) | (GET_BYTE(to_push, 3) >> 4), 12) * CAN_BMW_ACC_FAC;
    // if (ABS(bmw_lat_acc) > BMW_LAT_ACC_MAX) {
    //   print("Too big lateral acc \n");
    //   controls_allowed = false; //todo add soft-off request when violation occurs to loss of torque in the turn
    // }
  }

  // STEPPER_SERVO_CAN: get STEERING_STATUS
  if ((addr == 0x22f) && ((bus == BMW_F_CAN) || (bus == BMW_AUX_CAN))) {
    int torque_meas_new = ((float)(int8_t)(GET_BYTE(to_push, 2))); // torque raw
    update_sample(&torque_meas, torque_meas_new);

    if((((GET_BYTE(to_push, 1)>>4)>>CAN_ACTUATOR_CONTROL_STATUS_SOFTOFF_BIT) & 0x1) != 0x0){ //Soft off status means motor is shutting down due to error
      controls_allowed = false;
      print("BMW soft off\n");
    }
  }

  //get latest steering wheel angle rate
  if (addr == BMW_SteeringWheelAngle_slow) {
    float meas_angle = to_signed((GET_BYTE(to_push, 1) << 8) | GET_BYTE(to_push, 0), 16) * CAN_BMW_ANGLE_FAC; // deg
    // float angle_rate = to_signed((GET_BYTE(to_push, 4) << 8) | GET_BYTE(to_push, 3), 16) * CAN_BMW_ANGLE_FAC; // deg/s
    // // todo use common steer_angle_cmd_checks()
    // if(bmw_fmax_limit_check(meas_angle, bmw_max_angle, -bmw_max_angle)){
    //   // We should not be able to STEER under these conditions
    //   controls_allowed = false;
    //   if (cruise_engaged){
    //     print("Too big angle \n");
    //   }
    // }
    // if (meas_angle * bmw_rt_angle_last > 0.) { // ignore when zero crossing
    //   if (bmw_fmax_limit_check((meas_angle >= 0.) ? angle_rate : -angle_rate, angle_rate_up, -angle_rate_down)) { //should be sensitive for jerks to the outside
    //     controls_allowed = false;
    //     if (cruise_engaged){
    //       print("To fast angle rate \n");
    //     }
    //   }
    // }

    bmw_rt_angle_last = meas_angle;
  }

  // exit controls on brake press
  if (addr == BMW_EngineAndBrake) {
    brake_pressed = (GET_BYTE(to_push, 7) & 0x20U) != 0U;
  }

  if (addr == BMW_AccPedal) {
    gas_pressed = (GET_BYTE(to_push, 6) & 0x30U) != 0U;
  }

  generic_rx_checks(false);
}

static bool bmw_tx_hook(const CANPacket_t *to_send) {
  // const TorqueSteeringLimits BMW_STEERING_LIMITS = {
  //   .max_torque = 350,
  //   .max_rate_up = 3,
  //   .max_rate_down = 5,
  //   .max_rt_delta = 125,
  //   .type = TorqueMotorLimited,
  // };

  UNUSED(to_send);

  bool tx = true;
  int addr = GET_ADDR(to_send);
  static float bmw_desired_angle_last = 0; // last desired steer angle
  // STEPPER_SERVO_CAN: get STEERING_COMMAND
  // do not transmit CAN message if steering angle too high
  if (addr == 0x22e) {
    if (((GET_BYTE(to_send, 1) >> 4) & 0b11u) != 0x0){ //control enabled
      float steer_torque = ((float)(int8_t)(GET_BYTE(to_send, 4))) * CAN_ACTUATOR_TQ_FAC; //Nm
      // if (steer_torque_cmd_checks(steer_torque, -1, BMW_STEERING_LIMITS)
        if (bmw_fmax_limit_check(steer_torque - actuator_torque, max_tq_rate, -max_tq_rate)) {
        print("Violation torque rate");
        // printf("Tq: %f, ActTq: %f, Max: %f\n", steer_torque, actuator_torque, max_tq_rate);
        tx = false;
      }
    }
    float desired_angle = 0;
    if (((GET_BYTE(to_send, 1) >> 4) & 0b11u) == 0x2){ //position control enabled
      float angle_delta_req = ((float)(int16_t)((GET_BYTE(to_send, 2)) | (GET_BYTE(to_send, 3) << 8))) * CAN_ACTUATOR_POS_FAC; //deg/10ms
      desired_angle = bmw_rt_angle_last + angle_delta_req; //measured + requested delta

      if (controls_allowed == true) {
        bool violation = false;
        //check for max angles
        violation |= bmw_fmax_limit_check(desired_angle, bmw_max_angle, -bmw_max_angle);
        print("Violation desired angle");
        //angle is rate limited in carcontrols so it shouldn't exceed max delta
        float angle_delta_req_side = (bmw_desired_angle_last >= 0.) ? angle_delta_req : -angle_delta_req;
        violation |= bmw_fmax_limit_check(angle_delta_req_side, angle_rate_up, -angle_rate_down);
        print("Violation  delta");

        if (violation) {
          tx = false;
          desired_angle = bmw_desired_angle_last; //nothing was sent - hold to previous
        }
      }
    }
    bmw_desired_angle_last = desired_angle;
  }

  return tx;
}

static safety_config bmw_init(uint16_t param) {
  UNUSED(param);
  bmw_speed = 0;
  lever_position = -1;

  safety_config ret = BUILD_SAFETY_CFG(bmw_rx_checks, BMW_TX_MSGS);


  #ifdef ALLOW_DEBUG
    print("BMW safety init\n");
  #endif

  return ret;
}

const safety_hooks bmw_hooks = {
  .init = bmw_init,
  .rx = bmw_rx_hook,
  .tx = bmw_tx_hook,
  .get_counter = bmw_get_counter,
};
