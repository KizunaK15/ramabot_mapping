#include "config.h"

// ================= SETUP & AKTUASI MOTOR PWM =================
void setupMotorPWM() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(M1A, PWM_FREQ, PWM_RES); ledcAttach(M1B, PWM_FREQ, PWM_RES);
  ledcAttach(M2A, PWM_FREQ, PWM_RES); ledcAttach(M2B, PWM_FREQ, PWM_RES);
#else
  ledcSetup(CH_M1A, PWM_FREQ, PWM_RES); ledcAttachPin(M1A, CH_M1A);
  ledcSetup(CH_M1B, PWM_FREQ, PWM_RES); ledcAttachPin(M1B, CH_M1B);
  ledcSetup(CH_M2A, PWM_FREQ, PWM_RES); ledcAttachPin(M2A, CH_M2A);
  ledcSetup(CH_M2B, PWM_FREQ, PWM_RES); ledcAttachPin(M2B, CH_M2B);
#endif
}

void writeMotorChannel(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

int applyMotorDeadzone(float pwm) {
  int val = (int)roundf(pwm);
  if (val > 0 && val < MOTOR_MIN_PWM) return MOTOR_MIN_PWM;
  if (val < 0 && val > -MOTOR_MIN_PWM) return -MOTOR_MIN_PWM;
  return constrain(val, -255, 255);
}

void driveMotors(float pwmA, float pwmB) {
  if (!motorRunning || eStopActive || otaInProgress) {
    writeMotorChannel(M1A, CH_M1A, 0); writeMotorChannel(M1B, CH_M1B, 0);
    writeMotorChannel(M2A, CH_M2A, 0); writeMotorChannel(M2B, CH_M2B, 0);
    return;
  }

  int valA = applyMotorDeadzone(pwmA);
  if (valA >= 0) { writeMotorChannel(M1A, CH_M1A, valA); writeMotorChannel(M1B, CH_M1B, 0); }
  else           { writeMotorChannel(M1A, CH_M1A, 0);    writeMotorChannel(M1B, CH_M1B, -valA); }

  int valB = applyMotorDeadzone(pwmB);
  if (valB >= 0) { writeMotorChannel(M2A, CH_M2A, valB); writeMotorChannel(M2B, CH_M2B, 0); }
  else           { writeMotorChannel(M2A, CH_M2A, 0);    writeMotorChannel(M2B, CH_M2B, -valB); }
}

// ================= CORE 1: LOOP KONTROL & IMU PRESISI (200Hz) =================
void taskControlLoop(void *parameter) {
  unsigned long lastImuMicros = micros();
  unsigned long lastPidMicros = micros();
  int8_t prevSnapTurn = 0;

  for (;;) {
    if (otaInProgress) {
      driveMotors(0, 0);
      vTaskDelay(pdMS_TO_TICKS(20));
      lastImuMicros = micros();
      lastPidMicros = micros();
      continue;
    }

    unsigned long nowMicros = micros();
    float dtImu = (nowMicros - lastImuMicros) / 1000000.0f;
    if (dtImu <= 0.0f || dtImu > 0.1f) dtImu = 0.005f;
    lastImuMicros = nowMicros;

    // 1. Baca MPU6886 IMU & Thermal Bias Tracking
    M5.Imu.update();
    auto imu_data = M5.Imu.getImuData();
    accX = imu_data.accel.x; accY = imu_data.accel.y; accZ = imu_data.accel.z;
    gyroX = imu_data.gyro.x; gyroY = imu_data.gyro.y; gyroZ = imu_data.gyro.z;

    float rawDiffZ = gyroZ - gyroZOffset;
    bool motorsIdle = (fabsf(outputPWM_A) < 1.0f && fabsf(outputPWM_B) < 1.0f);

    if (motorsIdle && fabsf(rawDiffZ) < GYRO_NOISE_FLOOR_DPS) {
      gyroZOffset = (0.998f * gyroZOffset) + (0.002f * gyroZ);
      rawDiffZ = 0.0f;
    }

    float rateZ = GYRO_Z_SIGN * rawDiffZ;
    if (fabsf(rateZ) < GYRO_NOISE_FLOOR_DPS) {
      rateZ = 0.0f;
    }

    float deltaHeadingDeg = 0.5f * (rateZ + prevRateZ) * dtImu;
    prevRateZ = rateZ;

    portENTER_CRITICAL(&stateMux);
    headingDeg = normalizeAngleDeg(headingDeg + deltaHeadingDeg);
    portEXIT_CRITICAL(&stateMux);

    // 2. Loop PID Heading Lock & Odometri Gerak Algoritma Robot Utama (50Hz / 20ms)
    if (nowMicros - lastPidMicros >= 20000) {
      float dtPid = (nowMicros - lastPidMicros) / 1000000.0f;
      lastPidMicros = nowMicros;
      unsigned long nowMillis = millis();

      struct_joy_msg joySnap;
      portENTER_CRITICAL(&stateMux);
      if (nowMillis - lastJoyRecvTime > 500) {
        incomingJoy.linear_speed  = 0;
        incomingJoy.angular_speed = 0;
        incomingJoy.snap_turn     = 0;
      }
      if ((long)(nowMillis - rosCmdExpireTime) > 0) {
        rosCmdVx  = 0.0f;
        rosCmdVth = 0.0f;
      }
      if (webStepActive && nowMillis > webStepEndTime) {
        webStepActive = false;
      }
      joySnap      = incomingJoy;
      bool stepOn  = webStepActive;
      float stepV  = webStepTargetVel;
      float rVx    = rosCmdVx;
      float rVth   = rosCmdVth;
      portEXIT_CRITICAL(&stateMux);

      // Pembacaan delta pulsa kumulatif (Algoritma Robot Utama)
      noInterrupts();
      long curPulseL = pulseL;
      long curPulseR = pulseR;
      interrupts();

      long dPulseL = curPulseL - prevPulseL;
      long dPulseR = curPulseR - prevPulseR;
      prevPulseL = curPulseL;
      prevPulseR = curPulseR;

      float dL = (float)dPulseL * DIST_PER_PULSE * linearScale;
      float dR = (float)dPulseR * DIST_PER_PULSE * linearScale;
      float dS = (dR + dL) * 0.5f;
      float dThetaRad = deltaHeadingDeg * DEG_TO_RAD;

      estVx  = dS / dtPid;
      estVth = rateZ * DEG_TO_RAD;

      float maxPwmScale = joySnap.btn_boost ? 255.0f : (normalSpeedPct * 255.0f);
      float basePwmL = 0.0f;
      float basePwmR = 0.0f;
      bool isTurningCmd = false;

      if (stepOn) {
        float pwmStep = (stepV / maxLinearVel) * 255.0f;
        basePwmL = pwmStep;
        basePwmR = pwmStep;
      } else if (joySnap.linear_speed != 0 || joySnap.angular_speed != 0) {
        float linPwm = (joySnap.linear_speed / 100.0f) * maxPwmScale;
        float angPwm = (joySnap.angular_speed / 100.0f) * maxPwmScale;
        basePwmL = linPwm + angPwm;
        basePwmR = linPwm - angPwm;
        isTurningCmd = (joySnap.angular_speed != 0);
      } else if (fabsf(rVx) > 0.005f || fabsf(rVth) > 0.01f) {
        float vL = rVx - 0.5f * rVth * TRACK_WIDTH;
        float vR = rVx + 0.5f * rVth * TRACK_WIDTH;
        basePwmL = (vL / maxLinearVel) * 255.0f;
        basePwmR = (vR / maxLinearVel) * 255.0f;
        isTurningCmd = (fabsf(rVth) > 0.02f);
      }

      if (joySnap.snap_turn != 0 && prevSnapTurn == 0) {
        targetHeading = normalizeAngleDeg(targetHeading + (joySnap.snap_turn * 90.0f));
        yawErrSum = 0.0f;
        lastYawErr = 0.0f;
        yawLockedSettled = false;
      }
      prevSnapTurn = joySnap.snap_turn;

      bool isRobotIdle = (!stepOn &&
                          joySnap.linear_speed == 0 &&
                          joySnap.angular_speed == 0 &&
                          fabsf(rVx) <= 0.005f &&
                          fabsf(rVth) <= 0.01f);

      float localYawOut = 0.0f;
      if (headingLockEnabled && motorRunning && !eStopActive) {
        if (isTurningCmd) {
          targetHeading = headingDeg;
          yawErrSum = 0.0f;
          lastYawErr = 0.0f;
          yawLockedSettled = true;
          turnSettlingTimer = nowMillis;
        } else if (nowMillis - turnSettlingTimer < ENTER_PID_DELAY_MS) {
          targetHeading = headingDeg;
          yawErrSum = 0.0f;
          lastYawErr = 0.0f;
          yawLockedSettled = true;
        } else {
          float yawErr = normalizeAngleDeg(targetHeading - headingDeg);
          float absErr = fabsf(yawErr);

          if (isRobotIdle) {
            if (yawLockedSettled) {
              if (absErr > YAW_LOCK_EXIT_DEG) {
                yawLockedSettled = false;
              }
            } else {
              if (absErr <= YAW_LOCK_ENTER_DEG && fabsf(rateZ) < 1.5f) {
                yawLockedSettled = true;
              }
            }
          } else {
            yawLockedSettled = (absErr <= YAW_DEADBAND_MOVE_DEG);
          }

          if (yawLockedSettled) {
            yawErr = 0.0f;
            lastYawErr = 0.0f;
            localYawOut = 0.0f;
            if (nowMillis - yawZeroTimer >= YAW_AUTO_OFF_MS) {
              yawErrSum = 0.0f;
            }
          } else {
            yawZeroTimer = nowMillis;
            float stopMargin = isRobotIdle ? YAW_LOCK_ENTER_DEG : YAW_DEADBAND_MOVE_DEG;
            float effectiveErr = (yawErr > 0) ? max(0.0f, yawErr - stopMargin)
                                              : min(0.0f, yawErr + stopMargin);

            yawErrSum = constrain(yawErrSum + effectiveErr * dtPid, -80.0f, 80.0f);
            localYawOut = (Kp_yaw * effectiveErr) + (Ki_yaw * yawErrSum) - (Kd_yaw * rateZ);

            const float maxYawCorrPWM = 105.0f;
            if (Ki_yaw > 0.0001f) {
              if (localYawOut > maxYawCorrPWM)  yawErrSum -= (localYawOut - maxYawCorrPWM) / Ki_yaw;
              if (localYawOut < -maxYawCorrPWM) yawErrSum -= (localYawOut + maxYawCorrPWM) / Ki_yaw;
            }
            localYawOut = constrain(localYawOut, -maxYawCorrPWM, maxYawCorrPWM);
            lastYawErr = effectiveErr;

            if (isRobotIdle && fabsf(localYawOut) < YAW_MIN_ACTIVE_PWM) {
              localYawOut = 0.0f;
            }
          }
        }
      } else {
        targetHeading = headingDeg;
        yawErrSum = 0.0f;
        lastYawErr = 0.0f;
        yawLockedSettled = true;
      }

      if (motorRunning && !eStopActive) {
        outputPWM_A = constrain(basePwmL - localYawOut, -255.0f, 255.0f);
        outputPWM_B = constrain(basePwmR + localYawOut, -255.0f, 255.0f);
        driveMotors(outputPWM_A, outputPWM_B);
      } else {
        outputPWM_A = 0.0f;
        outputPWM_B = 0.0f;
        driveMotors(0, 0);
      }

      // Pembaruan Pose Odometri (Algoritma Robot Utama)
      portENTER_CRITICAL(&stateMux);
      yawPidOut = localYawOut;
      float thetaMid = odoTheta + dThetaRad * 0.5f;
      odoX += dS * cosf(thetaMid);
      odoY += dS * sinf(thetaMid);
      odoTheta = normalizeAngleRad(odoTheta + dThetaRad);
      portEXIT_CRITICAL(&stateMux);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}