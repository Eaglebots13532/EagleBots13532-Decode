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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Decode_TeleOp extends LinearOpMode {

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
  double limitSpeed = 0.8;
  double turnDeg = .5;
  public int maxFlyRPM = 1850;
  public int side = 20; // Blue

  @Override
  public void runOpMode() {

    try {
      // Initialize class components
      drive.init();
      odo.DoInit();
      decode.InitIL();
      april.initAprilTag();
      // Wait for the DS start button to be touched.
      telemetry.addLine("Basic control Ready");
      telemetry.update();
      waitForStart();
      //
      decode.Intake(); // start intake
      decode.closeGate(); // wait for Balls
      // decode.chuteAngle(130);//range in inches
      decode.spinUp(maxFlyRPM); // start flywheel to reduce current

      while (opModeIsActive()) {
        // drive by game pad
        // fieldx & fieldy are the drive position omega turn
        drive.fieldRelativeDrive(
            -gamepad1.left_stick_y * drive.maxSpeedMetersPerSec,
            -gamepad1.left_stick_x * drive.maxSpeedMetersPerSec,
            -gamepad1.right_stick_x * drive.maxOmegaRadPerSec);

        // if ready to launch set the speed according to distance
        if (gamepad2.leftBumperWasPressed()) {
          april.getAprilTag();
          side = DC_AprilTagLocalization.MetaId;
          double srange = april.getRange();
          // does it see april tag?
          if (april.getMetaId() == 0) {
            srange = 30;
          }
          if (srange > 100) srange = 120; // max rpm
          /*
          // shift robot to align with april tag
          double bshift =
              DC_AprilTagLocalization.range
                  * Math.sin(Math.toRadians(DC_AprilTagLocalization.bearing));
          runTime.reset();
          while (opModeIsActive()
              && runTime.seconds() < 3.0
              && DC_AprilTagLocalization.bearing > 1.0) {
            april.getAprilTag();
            // align robot to april with field oriented movements
            double pshift =
                DC_AprilTagLocalization.range
                    * Math.sin(Math.toRadians(DC_AprilTagLocalization.bearing));
            double sshift = pshift / bshift; // move right or left
            // if (side == 24) sshift = -sshift; // sign may need to be changed
            drive.fieldRelativeDrive(
                -0.0 * drive.maxSpeedMetersPerSec,
                -sshift * drive.maxSpeedMetersPerSec,
                -0.0 * drive.maxOmegaRadPerSec);
          }
          */
          decode.chuteAngle(srange);
          decode.flyVelocity(srange);
          decode.openGate();
          decode.Intake();
          sleep(1000); // wait for ball to launch
          decode.IntakeStop();
          decode.closeGate();
          sleep(500); // wait for gate to close
        } // end while adjust for bearing
        decode.Intake(); // start intake
        sleep(1000); // intake is running allow to release bumper
      }
    } // end try
    catch (Exception e) {
      telemetry.addLine(", exception in gamePadTeleOP");
      telemetry.addData("e", e);
      telemetry.update();
      sleep(2000);
      requestOpModeStop();
    } // catch exception
  } // run op mode
} // end class Swerve Components
