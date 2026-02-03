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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_aprilTagVerify extends LinearOpMode {

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

  // Game Pad controls
  double gpY = 0.0;
  double gpX = 0.0;
  double limitSpeed = 0.5;
  double turnDeg = .5;
  int side = 20; // Blue

  @Override
  public void runOpMode() {
    // Initialize class components
    drive.init();
    odo.DoInit();
    decode.InitIL();
    april.initAprilTag();

    // Wait for the DS start button to be touched.
    telemetry.addLine("Basic controlReady");
    telemetry.update();
    side = 20;
    waitForStart();
    while (opModeIsActive()) {
      try {
        /* x & y are the drive position
        drive.fieldRelativeDrive(
            -gamepad1.left_stick_y * drive.maxSpeedMetersPerSec,
            -gamepad1.left_stick_x * drive.maxSpeedMetersPerSec,
            -gamepad1.right_stick_x * drive.maxOmegaRadPerSec);

         */
        telemetry.addLine(". . . . . . . . . .");
        // if ready to launch set the speed according to distance
        april.getAprilTag();
        decode.chuteAngle(april.getRange());

        // align robot to april with field Oriented movements
        telemetry.addData(". . range", april.range);
        telemetry.addData("bearing", april.bearing);
        telemetry.addData("Red range", april.Rrange);
        telemetry.addData("Blue range", april.Brange);
        telemetry.update();
      } // end try
      catch (Exception e) {
        telemetry.addLine(", exception in gamePadTeleOP");
        telemetry.update();
        sleep(2000);
        requestOpModeStop();
      } // catch exception
    } // while op mode running
  } // run op mode
} // end class Swerve Components
