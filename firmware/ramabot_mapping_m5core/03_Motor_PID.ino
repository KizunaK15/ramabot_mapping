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

void driveMotors(float pwmA, float pwmB) {
  if (!motorRunning || eStopActive || otaInProgress) {
    writeMotorChannel(M1A, CH_M1A, 0); writeMotorChannel(M1B, CH_M1B, 0);
    writeMotorChannel(M2A, CH_M2A, 0); writeMotorChannel(M2B, CH_M2B, 0);
    return;
  }
  int valA = constrain((int)abs(pwmA), 0, 255);
  if (pwmA >= 0) { writeMotorChannel(M1A, CH_M1A, valA); writeMotorChannel(M1B, CH_M1B, 0); }
  else           { writeMotorChannel(M1A, CH_M1A, 0);    writeMotorChannel(M1B, CH_M1B, valA); }

  int valB = constrain((int)abs(pwmB), 0, 255);
  if (pwmB >= 0) { writeMotorChannel(M2A, CH_M2A, valB); writeMotorChannel(M2B, CH_M2B, 0); }
  else           { writeMotorChannel(M2A, CH_M2A, 0);    writeMotorChannel(M2B, CH_M2B, valB); }
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
    lastImuMicros = nowMicros;

    M5.Imu.update();
    auto imu_data = M5.Imu.getImuData();
    accX = imu_data.accel.x; accY = imu_data.accel.y; accZ = imu_data.accel.z;
    gyroX = imu_data.gyro.x; gyroY = imu_data.gyro.y; gyroZ = imu_data.gyro.z;
    magX = imu_data.mag.x;   magY = imu_data.mag.y;   magZ = imu_data.mag.z;

    float rateZ = GYRO_Z_SIGN * (gyroZ - gyroZOffset);
    if (fabsf(rateZ) < 0.45f) rateZ = 0.0f;
    float deltaHeadingDeg = rateZ * dtImu;

    portENTER_CRITICAL(&stateMux);
    headingDeg = normalizeAngleDeg(headingDeg + deltaHeadingDeg);
    portEXIT_CRITICAL(&stateMux);

    if (nowMicros - lastPidMicros >= 20000) {
      float dtPid = (nowMicros - lastPidMicros) / 1000000.0f;
      lastPidMicros = nowMicros;
      unsigned long nowMillis = millis();

      struct_joy_msg joySnap;
      portENTER_CRITICAL(&stateMux);
      if (nowMillis - lastJoyRecvTime > 500) {
        incomingJoy.linear_speed = 0;
        incomingJoy.angular_speed = 0;
        incomingJoy.btn_test_straight = false;
        incomingJoy.snap_turn = 0;
      }
      if (webStepActive && nowMillis > webStepEndTime) {
        webStepActive = false;
      }
      joySnap = incomingJoy;
      bool stepOn = webStepActive;
      float stepRpm = webStepTargetRPM;
      portEXIT_CRITICAL(&stateMux);

      noInterrupts();
      long curPulseA = pulseA;
      long curPulseB = pulseB;
      interrupts();

      long dPulseA = curPulseA - prevPulseA;
      long dPulseB = curPulseB - prevPulseB;
      prevPulseA = curPulseA;
      prevPulseB = curPulseB;

      float curRpmA = ((float)dPulseA / TOTAL_PPR_LEFT)  * (60.0f / dtPid);
      float curRpmB = ((float)dPulseB / TOTAL_PPR_RIGHT) * (60.0f / dtPid);

      float dL = ENC_SIGN_LEFT  * (float)dPulseA * DIST_PER_PULSE_LEFT;
      float dR = ENC_SIGN_RIGHT * (float)dPulseB * DIST_PER_PULSE_RIGHT;
      float dS = (dR + dL) * 0.5f;
      float dThetaRad = deltaHeadingDeg * DEG_TO_RAD;

      float lin_rpm = 0.0f;
      float ang_rpm = 0.0f;

      if (stepOn) {
        lin_rpm = stepRpm;
        ang_rpm = 0.0f;
      } else if (joySnap.btn_test_straight) {
        lin_rpm = 40.0f;
        ang_rpm = 0.0f;
      } else {
        lin_rpm = (joySnap.linear_speed / 100.0f) * maxRPM;
        ang_rpm = (joySnap.angular_speed / 100.0f) * maxRPM;
      }

      if (joySnap.snap_turn != 0 && prevSnapTurn == 0) {
        targetHeading = normalizeAngleDeg(targetHeading + (joySnap.snap_turn * 90.0f));
        yawErrSum = 0.0f;
        lastYawErr = 0.0f;
      }
      prevSnapTurn = joySnap.snap_turn;

      float localYawOut = 0.0f;
      if (headingLockEnabled && motorRunning && !eStopActive) {
        if (joySnap.angular_speed != 0) {
          targetHeading = headingDeg;
          yawErrSum = 0.0f;
          lastYawErr = 0.0f;
          turnSettlingTimer = nowMillis;
        } else if (nowMillis - turnSettlingTimer < ENTER_PID_DELAY_MS) {
          targetHeading = headingDeg;
          yawErrSum = 0.0f;
          lastYawErr = 0.0f;
        } else {
          float yawErr = normalizeAngleDeg(targetHeading - headingDeg);
          if (fabsf(yawErr) < YAW_DEADBAND_DEG) {
            yawErr = 0.0f;
          }

          if (yawErr == 0.0f) {
            if (nowMillis - yawZeroTimer >= YAW_AUTO_OFF_MS) {
              yawErrSum = 0.0f;
            }
          } else {
            yawZeroTimer = nowMillis;
          }

          yawErrSum = constrain(yawErrSum + yawErr * dtPid, -50.0f, 50.0f);
          float dYawErr = (yawErr - lastYawErr) / dtPid;
          localYawOut = (Kp_yaw * yawErr) + (Ki_yaw * yawErrSum) + (Kd_yaw * dYawErr);

          const float maxYawCorrRPM = 45.0f;
          if (Ki_yaw > 0.0001f) {
            if (localYawOut > maxYawCorrRPM)  yawErrSum -= (localYawOut - maxYawCorrRPM) / Ki_yaw;
            if (localYawOut < -maxYawCorrRPM) yawErrSum -= (localYawOut + maxYawCorrRPM) / Ki_yaw;
          }
          localYawOut = constrain(localYawOut, -maxYawCorrRPM, maxYawCorrRPM);
          lastYawErr = yawErr;
        }
      } else {
        targetHeading = headingDeg;
        yawErrSum = 0.0f;
        lastYawErr = 0.0f;
      }

      float tgtA = constrain(lin_rpm + ang_rpm - localYawOut, -maxRPM, maxRPM);
      float tgtB = constrain(lin_rpm - ang_rpm + localYawOut, -maxRPM, maxRPM);

      if (motorRunning && !eStopActive) {
        if (testMode == 0) {
          outputPWM_A = (tgtA / maxRPM) * 255.0f;
          outputPWM_B = (tgtB / maxRPM) * 255.0f;
        } else {
          if (tgtA == 0.0f && fabsf(curRpmA) < 2.0f) {
            outputPWM_A = 0.0f; errSum_A = 0.0f; lastErr_A = 0.0f;
          } else {
            float errA = tgtA - curRpmA;
            errSum_A = constrain(errSum_A + errA * dtPid, -255.0f, 255.0f);
            float dErrA = (errA - lastErr_A) / dtPid;
            outputPWM_A = constrain(outputPWM_A + (Kp * errA) + (Ki * errSum_A) + (Kd * dErrA), -255.0f, 255.0f);
            lastErr_A = errA;
          }

          if (tgtB == 0.0f && fabsf(curRpmB) < 2.0f) {
            outputPWM_B = 0.0f; errSum_B = 0.0f; lastErr_B = 0.0f;
          } else {
            float errB = tgtB - curRpmB;
            errSum_B = constrain(errSum_B + errB * dtPid, -255.0f, 255.0f);
            float dErrB = (errB - lastErr_B) / dtPid;
            outputPWM_B = constrain(outputPWM_B + (Kp * errB) + (Ki * errSum_B) + (Kd * dErrB), -255.0f, 255.0f);
            lastErr_B = errB;
          }
        }
        driveMotors(outputPWM_A, outputPWM_B);
      } else {
        driveMotors(0, 0);
        outputPWM_A = 0; outputPWM_B = 0;
        errSum_A = 0; errSum_B = 0;
      }

      portENTER_CRITICAL(&stateMux);
      rpmA = curRpmA; rpmB = curRpmB;
      targetRPM_A = tgtA; targetRPM_B = tgtB;
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
