#include "MarlinConfig.h"
#if ENABLED(I2C_POSITION_ENCODERS)
  #include "Marlin.h"
  #include "temperature.h"
  #include "stepper.h"
  #include "I2CPositionEncoder.h"
  #include "gcode.h"
  #include <Wire.h>
  void I2CPositionEncoder::init(const uint8_t address, const AxisEnum axis) {
    encoderAxis = axis;
    i2cAddress = address;
    initialised++;
    SERIAL_ECHOPAIR("Setting up encoder on ", axis_codes[encoderAxis]);
    SERIAL_ECHOLNPAIR(" axis, addr = ", address);
    position = get_position();
  }
  void I2CPositionEncoder::update() {
    if (!initialised || !homed || !active) return; 
    position = get_position();
    if (!passes_test(false)) { 
      lastErrorTime = millis();
      return;
    }
    if (!trusted) {
      return;
    }
    lastPosition = position;
    const millis_t positionTime = millis();
    if (ec && ecMethod != I2CPE_ECM_NONE) {
      #ifdef I2CPE_EC_THRESH_PROPORTIONAL
        const millis_t deltaTime = positionTime - lastPositionTime;
        const uint32_t distance = abs(position - lastPosition),
                       speed = distance / deltaTime;
        const float threshold = constrain((speed / 50), 1, 50) * ecThreshold;
      #else
        const float threshold = get_error_correct_threshold();
      #endif
      #if ENABLED(I2CPE_ERR_ROLLING_AVERAGE)
        float sum = 0, diffSum = 0;
        errIdx = (errIdx >= I2CPE_ERR_ARRAY_SIZE - 1) ? 0 : errIdx + 1;
        err[errIdx] = get_axis_error_steps(false);
        LOOP_L_N(i, I2CPE_ERR_ARRAY_SIZE) {
          sum += err[i];
          if (i) diffSum += abs(err[i-1] - err[i]);
        }
        const int32_t error = int32_t(sum / (I2CPE_ERR_ARRAY_SIZE + 1)); 
      #else
        const int32_t error = get_axis_error_steps(false);
      #endif
      #ifdef I2CPE_ERR_THRESH_ABORT
        if (labs(error) > I2CPE_ERR_THRESH_ABORT * planner.axis_steps_per_mm[encoderAxis]) {
          SERIAL_ECHOPGM("Axis error greater than set threshold, aborting!");
		  FunV006("Axis error greater than set threshold, aborting!");
          SERIAL_ECHOLN(error);
          safe_delay(5000);
        }
      #endif
      #if ENABLED(I2CPE_ERR_ROLLING_AVERAGE)
        if (errIdx == 0) {
          if (labs(error) > threshold * planner.axis_steps_per_mm[encoderAxis] &&
              diffSum < 10 * (I2CPE_ERR_ARRAY_SIZE - 1) && labs(error) < 2000) { 
            SERIAL_ECHO(axis_codes[encoderAxis]);
            SERIAL_ECHOPAIR(" diffSum: ", diffSum / (I2CPE_ERR_ARRAY_SIZE - 1));
            SERIAL_ECHOPAIR(" - err detected: ", error / planner.axis_steps_per_mm[encoderAxis]);
            SERIAL_ECHOLNPGM("mm; correcting!");
            thermalManager.babystepsTodo[encoderAxis] = -LROUND(error);
          }
        }
      #else
        if (labs(error) > threshold * planner.axis_steps_per_mm[encoderAxis]) {
          thermalManager.babystepsTodo[encoderAxis] = -LROUND(error/2);
        }
      #endif
      if (labs(error) > I2CPE_ERR_CNT_THRESH * planner.axis_steps_per_mm[encoderAxis]) {
        const millis_t ms = millis();
        if (ELAPSED(ms, nextErrorCountTime)) {
          SERIAL_ECHOPAIR("Large error on ", axis_codes[encoderAxis]);
          SERIAL_ECHOPAIR(" axis. error: ", (int)error);
          SERIAL_ECHOLNPAIR("; diffSum: ", diffSum);
          errorCount++;
          nextErrorCountTime = ms + I2CPE_ERR_CNT_DEBOUNCE_MS;
        }
      }
    }
    lastPositionTime = positionTime;
  }
  void I2CPositionEncoder::set_homed() {
    if (active) {
      reset();  
      delay(10);
      zeroOffset = get_raw_count();
      homed++;
      trusted++;
      #ifdef I2CPE_DEBUG
        SERIAL_ECHO(axis_codes[encoderAxis]);
        SERIAL_ECHOPAIR(" axis encoder homed, offset of ", zeroOffset);
        SERIAL_ECHOLNPGM(" ticks.");
      #endif
    }
  }
  bool I2CPositionEncoder::passes_test(const bool report) {
    if (report) {
      if (H != I2CPE_MAG_SIG_GOOD) SERIAL_ECHOPGM("Warning. ");
      SERIAL_ECHO(axis_codes[encoderAxis]);
      SERIAL_ECHOPGM(" axis ");
      serialprintPGM(H == I2CPE_MAG_SIG_BAD ? PSTR("magnetic strip ") : PSTR("encoder "));
      switch (H) {
        case I2CPE_MAG_SIG_GOOD:
        case I2CPE_MAG_SIG_MID:
          SERIAL_ECHOLNPGM("passes test; field strength ");
          serialprintPGM(H == I2CPE_MAG_SIG_GOOD ? PSTR("good.\n") : PSTR("fair.\n"));
          break;
        default:
          SERIAL_ECHOLNPGM("not detected!");
      }
    }
    return (H == I2CPE_MAG_SIG_GOOD || H == I2CPE_MAG_SIG_MID);
  }
  float I2CPositionEncoder::get_axis_error_mm(const bool report) {
    float target, actual, error;
    target = stepper.get_axis_position_mm(encoderAxis);
    actual = mm_from_count(position);
    error = actual - target;
    if (labs(error) > 10000) error = 0; 
    if (report) {
      SERIAL_ECHO(axis_codes[encoderAxis]);
      SERIAL_ECHOPAIR(" axis target: ", target);
      SERIAL_ECHOPAIR(", actual: ", actual);
      SERIAL_ECHOLNPAIR(", error : ",error);
    }
    return error;
  }
  int32_t I2CPositionEncoder::get_axis_error_steps(const bool report) {
    if (!active) {
      if (report) {
        SERIAL_ECHO(axis_codes[encoderAxis]);
        SERIAL_ECHOLNPGM(" axis encoder not active!");
      }
      return 0;
    }
    float stepperTicksPerUnit;
    int32_t encoderTicks = position, encoderCountInStepperTicksScaled;
    stepperTicksPerUnit = (type == I2CPE_ENC_TYPE_ROTARY) ? stepperTicks : planner.axis_steps_per_mm[encoderAxis];
    encoderCountInStepperTicksScaled = LROUND((stepperTicksPerUnit * encoderTicks) / encoderTicksPerUnit);
    int32_t target = stepper.position(encoderAxis),
            error = (encoderCountInStepperTicksScaled - target);
    bool suppressOutput = (labs(error - errorPrev) > 100);
    if (report) {
      SERIAL_ECHO(axis_codes[encoderAxis]);
      SERIAL_ECHOPAIR(" axis target: ", target);
      SERIAL_ECHOPAIR(", actual: ", encoderCountInStepperTicksScaled);
      SERIAL_ECHOLNPAIR(", error : ", error);
      if (suppressOutput) SERIAL_ECHOLNPGM("Discontinuity detected, suppressing error.");
    }
    errorPrev = error;
    return (suppressOutput ? 0 : error);
  }
  int32_t I2CPositionEncoder::get_raw_count() {
    uint8_t index = 0;
    i2cLong encoderCount;
    encoderCount.val = 0x00;
    if (Wire.requestFrom((int)i2cAddress, 3) != 3) {
      H = I2CPE_MAG_SIG_NF;
      return 0;
    }
    while (Wire.available())
      encoderCount.bval[index++] = (uint8_t)Wire.read();
    H = (B00000011 & (encoderCount.bval[2] >> 6));
    encoderCount.val = (encoderCount.bval[2] & B00100000) ? (encoderCount.val | 0xFFC00000) : (encoderCount.val & 0x003FFFFF);
    if (invert) encoderCount.val *= -1;
    return encoderCount.val;
  }
  bool I2CPositionEncoder::test_axis() {
    if (!(encoderAxis == X_AXIS || encoderAxis == Y_AXIS || encoderAxis == Z_AXIS)) return false;
    float startCoord[NUM_AXIS] = { 0 }, endCoord[NUM_AXIS] = { 0 };
    const float startPosition = soft_endstop_min[encoderAxis] + 10,
                endPosition = soft_endstop_max[encoderAxis] - 10,
                feedrate = FLOOR(MMM_TO_MMS((encoderAxis == Z_AXIS) ? HOMING_FEEDRATE_Z : HOMING_FEEDRATE_XY));
    ec = false;
    LOOP_NA(i) {
      startCoord[i] = stepper.get_axis_position_mm((AxisEnum)i);
      endCoord[i] = stepper.get_axis_position_mm((AxisEnum)i);
    }
    startCoord[encoderAxis] = startPosition;
    endCoord[encoderAxis] = endPosition;
    stepper.synchronize();
    planner.buffer_line(startCoord[X_AXIS],startCoord[Y_AXIS],startCoord[Z_AXIS],
                        stepper.get_axis_position_mm(E_AXIS), feedrate, 0);
    stepper.synchronize();
    if (!trusted) {
      int32_t startWaitingTime = millis();
      while (!trusted && millis() - startWaitingTime < I2CPE_TIME_TRUSTED)
        safe_delay(500);
    }
    if (trusted) { 
      planner.buffer_line(endCoord[X_AXIS], endCoord[Y_AXIS], endCoord[Z_AXIS],
                          stepper.get_axis_position_mm(E_AXIS), feedrate, 0);
      stepper.synchronize();
    }
    return trusted;
  }
  void I2CPositionEncoder::calibrate_steps_mm(const uint8_t iter) {
    if (type != I2CPE_ENC_TYPE_LINEAR) {
      SERIAL_ECHOLNPGM("Steps per mm calibration is only available using linear encoders.");
      return;
    }
    if (!(encoderAxis == X_AXIS || encoderAxis == Y_AXIS || encoderAxis == Z_AXIS)) {
      SERIAL_ECHOLNPGM("Automatic steps / mm calibration not supported for this axis.");
      return;
    }
    float old_steps_mm, new_steps_mm,
          startDistance, endDistance,
          travelDistance, travelledDistance, total = 0,
          startCoord[NUM_AXIS] = { 0 }, endCoord[NUM_AXIS] = { 0 };
    float feedrate;
    int32_t startCount, stopCount;
    feedrate = MMM_TO_MMS((encoderAxis == Z_AXIS) ? HOMING_FEEDRATE_Z : HOMING_FEEDRATE_XY);
    bool oldec = ec;
    ec = false;
    startDistance = 20;
    endDistance = soft_endstop_max[encoderAxis] - 20;
    travelDistance = endDistance - startDistance;
    LOOP_NA(i) {
      startCoord[i] = stepper.get_axis_position_mm((AxisEnum)i);
      endCoord[i] = stepper.get_axis_position_mm((AxisEnum)i);
    }
    startCoord[encoderAxis] = startDistance;
    endCoord[encoderAxis] = endDistance;
    LOOP_L_N(i, iter) {
      stepper.synchronize();
      planner.buffer_line(startCoord[X_AXIS],startCoord[Y_AXIS],startCoord[Z_AXIS],
                          stepper.get_axis_position_mm(E_AXIS), feedrate, 0);
      stepper.synchronize();
      delay(250);
      startCount = get_position();
      planner.buffer_line(endCoord[X_AXIS],endCoord[Y_AXIS],endCoord[Z_AXIS],
                          stepper.get_axis_position_mm(E_AXIS), feedrate, 0);
      stepper.synchronize();
      delay(250);
      stopCount = get_position();
      travelledDistance = mm_from_count(abs(stopCount - startCount));
      SERIAL_ECHOPAIR("Attempted to travel: ", travelDistance);
      SERIAL_ECHOLNPGM("mm.");
      SERIAL_ECHOPAIR("Actually travelled:  ", travelledDistance);
      SERIAL_ECHOLNPGM("mm.");
      old_steps_mm = planner.axis_steps_per_mm[encoderAxis];
      new_steps_mm = (old_steps_mm * travelDistance) / travelledDistance;
      SERIAL_ECHOLNPAIR("Old steps per mm: ", old_steps_mm);
      SERIAL_ECHOLNPAIR("New steps per mm: ", new_steps_mm);
      planner.axis_steps_per_mm[encoderAxis] = new_steps_mm;
      if (iter > 1) {
        total += new_steps_mm;
        float tempCoord = startCoord[encoderAxis];
        startCoord[encoderAxis] = endCoord[encoderAxis];
        endCoord[encoderAxis] = tempCoord;
      }
    }
    if (iter > 1) {
      total /= (float)iter;
      SERIAL_ECHOLNPAIR("Average steps per mm: ", total);
    }
    ec = oldec;
    SERIAL_ECHOLNPGM("Calculated steps per mm has been set. Please save to EEPROM (M500) if you wish to keep these values.");
  }
  void I2CPositionEncoder::reset() {
    Wire.beginTransmission(i2cAddress);
    Wire.write(I2CPE_RESET_COUNT);
    Wire.endTransmission();
    #if ENABLED(I2CPE_ERR_ROLLING_AVERAGE)
      ZERO(err);
    #endif
  }
  bool I2CPositionEncodersMgr::I2CPE_anyaxis;
  uint8_t I2CPositionEncodersMgr::I2CPE_addr,
          I2CPositionEncodersMgr::I2CPE_idx;
  I2CPositionEncoder I2CPositionEncodersMgr::encoders[I2CPE_ENCODER_CNT];
  void I2CPositionEncodersMgr::init() {
    Wire.begin();
    #if I2CPE_ENCODER_CNT > 0
      uint8_t i = 0;
      encoders[i].init(I2CPE_ENC_1_ADDR, I2CPE_ENC_1_AXIS);
      #ifdef I2CPE_ENC_1_TYPE
        encoders[i].set_type(I2CPE_ENC_1_TYPE);
      #endif
      #ifdef I2CPE_ENC_1_TICKS_UNIT
        encoders[i].set_ticks_unit(I2CPE_ENC_1_TICKS_UNIT);
      #endif
      #ifdef I2CPE_ENC_1_TICKS_REV
        encoders[i].set_stepper_ticks(I2CPE_ENC_1_TICKS_REV);
      #endif
      #ifdef I2CPE_ENC_1_INVERT
        encoders[i].set_inverted(I2CPE_ENC_1_INVERT);
      #endif
      #ifdef I2CPE_ENC_1_EC_METHOD
        encoders[i].set_ec_method(I2CPE_ENC_1_EC_METHOD);
      #endif
      #ifdef I2CPE_ENC_1_EC_THRESH
        encoders[i].set_ec_threshold(I2CPE_ENC_1_EC_THRESH);
      #endif
      encoders[i].set_active(encoders[i].passes_test(true));
      #if I2CPE_ENC_1_AXIS == E_AXIS
        encoders[i].set_homed();
      #endif
    #endif
    #if I2CPE_ENCODER_CNT > 1
      i++;
      encoders[i].init(I2CPE_ENC_2_ADDR, I2CPE_ENC_2_AXIS);
      #ifdef I2CPE_ENC_2_TYPE
        encoders[i].set_type(I2CPE_ENC_2_TYPE);
      #endif
      #ifdef I2CPE_ENC_2_TICKS_UNIT
        encoders[i].set_ticks_unit(I2CPE_ENC_2_TICKS_UNIT);
      #endif
      #ifdef I2CPE_ENC_2_TICKS_REV
        encoders[i].set_stepper_ticks(I2CPE_ENC_2_TICKS_REV);
      #endif
      #ifdef I2CPE_ENC_2_INVERT
        encoders[i].set_inverted(I2CPE_ENC_2_INVERT);
      #endif
      #ifdef I2CPE_ENC_2_EC_METHOD
        encoders[i].set_ec_method(I2CPE_ENC_2_EC_METHOD);
      #endif
      #ifdef I2CPE_ENC_2_EC_THRESH
        encoders[i].set_ec_threshold(I2CPE_ENC_2_EC_THRESH);
      #endif
      encoders[i].set_active(encoders[i].passes_test(true));
      #if I2CPE_ENC_2_AXIS == E_AXIS
        encoders[i].set_homed();
      #endif
    #endif
    #if I2CPE_ENCODER_CNT > 2
      i++;
      encoders[i].init(I2CPE_ENC_3_ADDR, I2CPE_ENC_3_AXIS);
      #ifdef I2CPE_ENC_3_TYPE
        encoders[i].set_type(I2CPE_ENC_3_TYPE);
      #endif
      #ifdef I2CPE_ENC_3_TICKS_UNIT
        encoders[i].set_ticks_unit(I2CPE_ENC_3_TICKS_UNIT);
      #endif
      #ifdef I2CPE_ENC_3_TICKS_REV
        encoders[i].set_stepper_ticks(I2CPE_ENC_3_TICKS_REV);
      #endif
      #ifdef I2CPE_ENC_3_INVERT
        encoders[i].set_inverted(I2CPE_ENC_3_INVERT);
      #endif
      #ifdef I2CPE_ENC_3_EC_METHOD
        encoders[i].set_ec_method(I2CPE_ENC_3_EC_METHOD);
      #endif
      #ifdef I2CPE_ENC_3_EC_THRESH
        encoders[i].set_ec_threshold(I2CPE_ENC_3_EC_THRESH);
      #endif
    encoders[i].set_active(encoders[i].passes_test(true));
      #if I2CPE_ENC_3_AXIS == E_AXIS
        encoders[i].set_homed();
      #endif
    #endif
    #if I2CPE_ENCODER_CNT > 3
      i++;
      encoders[i].init(I2CPE_ENC_4_ADDR, I2CPE_ENC_4_AXIS);
      #ifdef I2CPE_ENC_4_TYPE
        encoders[i].set_type(I2CPE_ENC_4_TYPE);
      #endif
      #ifdef I2CPE_ENC_4_TICKS_UNIT
        encoders[i].set_ticks_unit(I2CPE_ENC_4_TICKS_UNIT);
      #endif
      #ifdef I2CPE_ENC_4_TICKS_REV
        encoders[i].set_stepper_ticks(I2CPE_ENC_4_TICKS_REV);
      #endif
      #ifdef I2CPE_ENC_4_INVERT
        encoders[i].set_inverted(I2CPE_ENC_4_INVERT);
      #endif
      #ifdef I2CPE_ENC_4_EC_METHOD
        encoders[i].set_ec_method(I2CPE_ENC_4_EC_METHOD);
      #endif
      #ifdef I2CPE_ENC_4_EC_THRESH
        encoders[i].set_ec_threshold(I2CPE_ENC_4_EC_THRESH);
      #endif
      encoders[i].set_active(encoders[i].passes_test(true));
      #if I2CPE_ENC_4_AXIS == E_AXIS
        encoders[i].set_homed();
      #endif
    #endif
    #if I2CPE_ENCODER_CNT > 4
      i++;
      encoders[i].init(I2CPE_ENC_5_ADDR, I2CPE_ENC_5_AXIS);
      #ifdef I2CPE_ENC_5_TYPE
        encoders[i].set_type(I2CPE_ENC_5_TYPE);
      #endif
      #ifdef I2CPE_ENC_5_TICKS_UNIT
        encoders[i].set_ticks_unit(I2CPE_ENC_5_TICKS_UNIT);
      #endif
      #ifdef I2CPE_ENC_5_TICKS_REV
        encoders[i].set_stepper_ticks(I2CPE_ENC_5_TICKS_REV);
      #endif
      #ifdef I2CPE_ENC_5_INVERT
        encoders[i].set_inverted(I2CPE_ENC_5_INVERT);
      #endif
      #ifdef I2CPE_ENC_5_EC_METHOD
        encoders[i].set_ec_method(I2CPE_ENC_5_EC_METHOD);
      #endif
      #ifdef I2CPE_ENC_5_EC_THRESH
        encoders[i].set_ec_threshold(I2CPE_ENC_5_EC_THRESH);
      #endif
      encoders[i].set_active(encoders[i].passes_test(true));
      #if I2CPE_ENC_5_AXIS == E_AXIS
        encoders[i].set_homed();
      #endif
    #endif
  }
  void I2CPositionEncodersMgr::report_position(const int8_t idx, const bool units, const bool noOffset) {
    CHECK_IDX();
    if (units)
      SERIAL_ECHOLN(noOffset ? encoders[idx].mm_from_count(encoders[idx].get_raw_count()) : encoders[idx].get_position_mm());
    else {
      if (noOffset) {
        const int32_t raw_count = encoders[idx].get_raw_count();
        SERIAL_ECHO(axis_codes[encoders[idx].get_axis()]);
        SERIAL_CHAR(' ');
        for (uint8_t j = 31; j > 0; j--)
          SERIAL_ECHO((bool)(0x00000001 & (raw_count >> j)));
        SERIAL_ECHO((bool)(0x00000001 & raw_count));
        SERIAL_CHAR(' ');
        SERIAL_ECHOLN(raw_count);
      }
      else
        SERIAL_ECHOLN(encoders[idx].get_position());
    }
  }
  void I2CPositionEncodersMgr::change_module_address(const uint8_t oldaddr, const uint8_t newaddr) {
    Wire.beginTransmission(newaddr);
    if (!Wire.endTransmission()) {
      SERIAL_ECHOPAIR("?There is already a device with that address on the I2C bus! (", newaddr);
      SERIAL_ECHOLNPGM(")");
      return;
    }
    Wire.beginTransmission(oldaddr);
    if (Wire.endTransmission()) {
      SERIAL_ECHOPAIR("?No module detected at this address! (", oldaddr);
      SERIAL_ECHOLNPGM(")");
      return;
    }
    SERIAL_ECHOPAIR("Module found at ", oldaddr);
    SERIAL_ECHOLNPAIR(", changing address to ", newaddr);
    Wire.beginTransmission(oldaddr);
    Wire.write(I2CPE_SET_ADDR);
    Wire.write(newaddr);
    Wire.endTransmission();
    SERIAL_ECHOLNPGM("Address changed, resetting and waiting for confirmation..");
    safe_delay(I2CPE_REBOOT_TIME);
    Wire.beginTransmission(newaddr);
    if (Wire.endTransmission()) {
      SERIAL_ECHOLNPGM("Address change failed! Check encoder module.");
      return;
    }
    SERIAL_ECHOLNPGM("Address change successful!");
    const int8_t idx = idx_from_addr(newaddr);
    if (idx >= 0 && !encoders[idx].get_active()) {
      SERIAL_ECHO(axis_codes[encoders[idx].get_axis()]);
      SERIAL_ECHOLNPGM(" axis encoder was not detected on printer startup. Trying again.");
      encoders[idx].set_active(encoders[idx].passes_test(true));
    }
  }
  void I2CPositionEncodersMgr::report_module_firmware(const uint8_t address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission()) {
      SERIAL_ECHOPAIR("?No module detected at this address! (", address);
      SERIAL_ECHOLNPGM(")");
      return;
    }
    SERIAL_ECHOPAIR("Requesting version info from module at address ", address);
    SERIAL_ECHOLNPGM(":");
    Wire.beginTransmission(address);
    Wire.write(I2CPE_SET_REPORT_MODE);
    Wire.write(I2CPE_REPORT_VERSION);
    Wire.endTransmission();
    if (Wire.requestFrom((int)address, 32)) {
      char c;
      while (Wire.available() > 0 && (c = (char)Wire.read()) > 0)
        SERIAL_ECHO(c);
      SERIAL_EOL();
    }
    Wire.beginTransmission(address);
    Wire.write(I2CPE_SET_REPORT_MODE);
    Wire.write(I2CPE_REPORT_DISTANCE);
    Wire.endTransmission();
  }
  int8_t I2CPositionEncodersMgr::parse() {
    I2CPE_addr = 0;
    if (parser.seen('A')) {
      if (!parser.has_value()) {
        SERIAL_PROTOCOLLNPGM("?A seen, but no address specified! [30-200]");
        return I2CPE_PARSE_ERR;
      };
      I2CPE_addr = parser.value_byte();
      if (!WITHIN(I2CPE_addr, 30, 200)) { 
        SERIAL_PROTOCOLLNPGM("?Address out of range. [30-200]");
        return I2CPE_PARSE_ERR;
      }
      I2CPE_idx = idx_from_addr(I2CPE_addr);
      if (I2CPE_idx >= I2CPE_ENCODER_CNT) {
        SERIAL_PROTOCOLLNPGM("?No device with this address!");
        return I2CPE_PARSE_ERR;
      }
    }
    else if (parser.seenval('I')) {
      if (!parser.has_value()) {
        SERIAL_PROTOCOLLNPAIR("?I seen, but no index specified! [0-", I2CPE_ENCODER_CNT - 1);
        SERIAL_PROTOCOLLNPGM("]");
        return I2CPE_PARSE_ERR;
      };
      I2CPE_idx = parser.value_byte();
      if (I2CPE_idx >= I2CPE_ENCODER_CNT) {
        SERIAL_PROTOCOLLNPAIR("?Index out of range. [0-", I2CPE_ENCODER_CNT - 1);
        SERIAL_ECHOLNPGM("]");
        return I2CPE_PARSE_ERR;
      }
      I2CPE_addr = encoders[I2CPE_idx].get_address();
    }
    else
      I2CPE_idx = 0xFF;
    I2CPE_anyaxis = parser.seen_axis();
    return I2CPE_PARSE_OK;
  };
  void I2CPositionEncodersMgr::M860() {
    if (parse()) return;
    const bool hasU = parser.seen('U'), hasO = parser.seen('O');
    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) report_position(idx, hasU, hasO);
        }
      }
    }
    else
      report_position(I2CPE_idx, hasU, hasO);
  }
  void I2CPositionEncodersMgr::M861() {
    if (parse()) return;
    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) report_status(idx);
        }
      }
    }
    else
      report_status(I2CPE_idx);
  }
  void I2CPositionEncodersMgr::M862() {
    if (parse()) return;
    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) test_axis(idx);
        }
      }
    }
    else
      test_axis(I2CPE_idx);
  }
  void I2CPositionEncodersMgr::M863() {
    if (parse()) return;
    const uint8_t iterations = constrain(parser.byteval('P', 1), 1, 10);
    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) calibrate_steps_mm(idx, iterations);
        }
      }
    }
    else
      calibrate_steps_mm(I2CPE_idx, iterations);
  }
  void I2CPositionEncodersMgr::M864() {
    uint8_t newAddress;
    if (parse()) return;
    if (!I2CPE_addr) I2CPE_addr = I2CPE_PRESET_ADDR_X;
    if (parser.seen('S')) {
      if (!parser.has_value()) {
        SERIAL_PROTOCOLLNPGM("?S seen, but no address specified! [30-200]");
        return;
      };
      newAddress = parser.value_byte();
      if (!WITHIN(newAddress, 30, 200)) {
        SERIAL_PROTOCOLLNPGM("?New address out of range. [30-200]");
        return;
      }
    }
    else if (!I2CPE_anyaxis) {
      SERIAL_PROTOCOLLNPGM("?You must specify S or [XYZE].");
      return;
    }
    else {
           if (parser.seen('X')) newAddress = I2CPE_PRESET_ADDR_X;
      else if (parser.seen('Y')) newAddress = I2CPE_PRESET_ADDR_Y;
      else if (parser.seen('Z')) newAddress = I2CPE_PRESET_ADDR_Z;
      else if (parser.seen('E')) newAddress = I2CPE_PRESET_ADDR_E;
      else return;
    }
    SERIAL_ECHOPAIR("Changing module at address ", I2CPE_addr);
    SERIAL_ECHOLNPAIR(" to address ", newAddress);
    change_module_address(I2CPE_addr, newAddress);
  }
  void I2CPositionEncodersMgr::M865() {
    if (parse()) return;
    if (!I2CPE_addr) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) report_module_firmware(encoders[idx].get_address());
        }
      }
    }
    else
      report_module_firmware(I2CPE_addr);
  }
  void I2CPositionEncodersMgr::M866() {
    if (parse()) return;
    const bool hasR = parser.seen('R');
    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) {
            if (hasR)
              reset_error_count(idx, AxisEnum(i));
            else
              report_error_count(idx, AxisEnum(i));
          }
        }
      }
    }
    else if (hasR)
      reset_error_count(I2CPE_idx, encoders[I2CPE_idx].get_axis());
    else
      report_error_count(I2CPE_idx, encoders[I2CPE_idx].get_axis());
  }
  void I2CPositionEncodersMgr::M867() {
    if (parse()) return;
    const int8_t onoff = parser.seenval('S') ? parser.value_int() : -1;
    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) {
            const bool ena = onoff == -1 ? !encoders[I2CPE_idx].get_ec_enabled() : !!onoff;
            enable_ec(idx, ena, AxisEnum(i));
          }
        }
      }
    }
    else {
      const bool ena = onoff == -1 ? !encoders[I2CPE_idx].get_ec_enabled() : !!onoff;
      enable_ec(I2CPE_idx, ena, encoders[I2CPE_idx].get_axis());
    }
  }
  void I2CPositionEncodersMgr::M868() {
    if (parse()) return;
    const float newThreshold = parser.seenval('T') ? parser.value_float() : -9999;
    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) {
            if (newThreshold != -9999)
              set_ec_threshold(idx, newThreshold, encoders[idx].get_axis());
            else
              get_ec_threshold(idx, encoders[idx].get_axis());
          }
        }
      }
    }
    else if (newThreshold != -9999)
      set_ec_threshold(I2CPE_idx, newThreshold, encoders[I2CPE_idx].get_axis());
    else
      get_ec_threshold(I2CPE_idx, encoders[I2CPE_idx].get_axis());
  }
  void I2CPositionEncodersMgr::M869() {
    if (parse()) return;
    if (I2CPE_idx == 0xFF) {
      LOOP_XYZE(i) {
        if (!I2CPE_anyaxis || parser.seen(axis_codes[i])) {
          const uint8_t idx = idx_from_axis(AxisEnum(i));
          if ((int8_t)idx >= 0) report_error(idx);
        }
      }
    }
    else
      report_error(I2CPE_idx);
  }
#endif 
