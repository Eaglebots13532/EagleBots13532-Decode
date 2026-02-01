// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

/* Copyright (c) 2024-2025 FTC 13532
-- All rights reserved.
- - - - -  eaglebots configuration - - - - - - - -
Left   Motor  0 . . . . . . . . .  LFM
Right  Motor  1 . . . . . . . . .  RFM
Left   Servo  0 . . . . . . . . .  LFS
Right  Servo  1 . . . . . . . . .  RFS

Left   Analog  . . . . .  Analog 0 LFP [one cable
Right  Analog  . . . . . .Analog 1 RFP  for both]
armpot Analog  . . . . . .Analog 2 armPot

Arm    Motor  2 . . . . . . . . . . arm
Spin   Motor  3 . . . . . . . . . . launch
Gate   Servo  4 . . . . . . . . . . gate
Intake Servo  2 . . . . . . . . . . intake
Guide  Servo  3 . . . . . . . . . . guide

Ball   Rev Color Sensor . . . IC1 0 ballSensor
Ball 1 ???
Ball 2 ???
Husky Camera   . . . . . . . .IC2 0 huskyLens
PinPoint . . . . . . . . . . .IC3 0 odo
April Camera   . . . . . . . . . . Webcam 1
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Decode_Autonomous extends LinearOpMode {

  // Swerve Devices
  // -----------------------------
  DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  // pinpoint
  DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
  // Game devices
  // -----------------------------
  // Husky Camera - ball objects
  // DC_Husky_Sensor ball = new DC_Husky_Sensor("color", this);
  // Front Camera - April tag
  // DC_Husky_Sensor tag = new DC_Husky_Sensor("april", this);
  // decode motors/servos
  DC_Intake_Launch decode = new DC_Intake_Launch(this);
  DC_AprilTagLocalization april = new DC_AprilTagLocalization(this);
  ElapsedTime runTime = new ElapsedTime();

  // Game Pad controls
  double gpY = 0.0;
  double gpX = 0.0;
  double limitSpeed = 0.5;
  double turnDeg = .5;
  int side = 20; // Blue

  @Override
  public void runOpMode() {
    waitForStart();
    try {

      // Initialize class components
      drive.init();
      odo.DoInit();
      decode.InitIL();
      april.initAprilTag();
      april.getAprilTag();
      if (april.MetaId == 24) side = 24;
      else side = 20;

      // Wait for the DS start button to be touched.
      telemetry.addLine("Autonomous Ready:" + april.MetaName);
      telemetry.update();
      // set ange
      april.getAprilTag();
      sleep(100); // wait for april response
      decode.chuteAngle(april.range); // range in inches
      decode.spinUp(april.range); // start flywheel to reduce current

      // adjust for bearing, may be able to remove this adjust code
      while (opModeIsActive()) {
        april.getAprilTag();
        double bshift = april.range * Math.sin(Math.toRadians(april.bearing));
        runTime.reset();
        while (opModeIsActive() && runTime.seconds() < 3.0 && april.bearing > 1.0) {
          april.getAprilTag();
          // align robot to april with field oriented movements
          double pshift = april.range * Math.sin(Math.toRadians(april.bearing));
          double sshift =
              pshift / bshift; // move right or left to center bearing, May need to reverse
          // if (side == 24) sshift = -sshift; // move left if read
          drive.fieldRelativeDrive(
              -0.0 * drive.maxSpeedMetersPerSec,
              -sshift * drive.maxSpeedMetersPerSec,
              -0.0 * drive.maxOmegaRadPerSec);
        } // end while adjust for bearing
        for (int i = 1; i < 4; i++) {
          decode.openGate();
          decode.Intake();
          sleep(1000); // wait for ball to launch
          decode.IntakeStop();
          decode.closeGate();
          sleep(500); // wait for gate to close
        }
        telemetry.update();
        decode.Intake(); // start intake
        sleep(1000);
      } // shoot 3 balls
      decode.IntakeStop();
      decode.spinOff();
      drive.fieldRelativeDrive(
          -10.0 * drive.maxSpeedMetersPerSec,
          -(-10.0) * drive.maxSpeedMetersPerSec,
          -0.0 * drive.maxOmegaRadPerSec);
      sleep(1000); // robot off launch area
      drive.fieldRelativeDrive(
          -0.0 * drive.maxSpeedMetersPerSec,
          -0.0 * drive.maxSpeedMetersPerSec,
          -0.0 * drive.maxOmegaRadPerSec);
    } // end try
    catch (Exception e) {
      telemetry.addLine(", exception in gamePadTeleOP");
      telemetry.update();
      sleep(2000);
      requestOpModeStop();
    } // catch exception
  } // run op mode
} // end class Swerve Components
