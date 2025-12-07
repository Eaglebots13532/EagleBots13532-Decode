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
public class UT_Basic_Decode_TeleOp extends LinearOpMode {

  // Swerve Devices
  // -----------------------------
  DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  // pinpoint
  DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
  // Game devices
  // -----------------------------
  // Husky Camera - ball objects
  DC_Husky_Sensor ball = new DC_Husky_Sensor("color", this);
  // Front Camera - April tag
  DC_Husky_Sensor tag = new DC_Husky_Sensor("april", this);
  // decode motors/servos
  DC_Intake_Launch decode = new DC_Intake_Launch(this);

  // ball sense
  DC_BallSensor present = new DC_BallSensor(this);

  // Game Pad controls
  double gpY = 0.0;
  double gpX = 0.0;
  double limitSpeed = 0.5;
  double turnDeg = .5;
  // toggles
  boolean upToggle = false;
  boolean leftToggle = false;
  boolean downToggle = false;
  boolean rightToggle = false;
  boolean btoggle = false;

  @Override
  public void runOpMode() {
    // Initialize class components
    drive.SwerveInit();
    odo.DoInit();
    decode.InitIL();
    ball.initHuskyLens();
    present.SensorInit();
    // Wait for the DS start button to be touched.
    telemetry.addLine("Basic controlReady");
    telemetry.update();
    // ready: raise to accept ball
    if (!decode.armPosition(0)) telemetry.addLine("Arm cmd invalid"); // ready: raise to accept ball
    telemetry.addLine("- - Game Pad A - -");
    telemetry.addLine("left stick Y drive");
    telemetry.addLine("right stick X turn");
    telemetry.addLine("B speed 50% or 80%");
    telemetry.addLine("- - Game Pad B - -");
    telemetry.addLine(" Left  Pad - intake on");
    telemetry.addLine("  Up   Pad - spin up");
    telemetry.addLine(" Right Pad - gate");
    telemetry.addLine(" Down  Pad - launch");
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      try {
        // x & y are the drive position
        gpY = -gamepad1.left_stick_y;
        gpX = gamepad1.right_stick_x;
        double speed = gpY * limitSpeed;
        drive.driveSpeed(speed);
        turnDeg = (gpX + 1.0) / 2.0;
        drive.servoTurn(turnDeg);
        // toggles
        if (B()) btoggle = !btoggle;
        if (B()) limitSpeed = .8;
        else limitSpeed = .5;
        if (dUp()) {
          if (upToggle) upToggle = !upToggle;
        }
        if (upToggle) {
          // change spin-up velocity
          while (opModeIsActive() && decode.spinUp(4500.0)) {
            idle();
          }
        } else decode.spinOff();
        if (dLeft()) {
          if (leftToggle) leftToggle = !leftToggle;
        }
        if (leftToggle) {
          decode.Intake(); // this is 0 or 1
        } else decode.Intake(); // stop intake
        if (dRight()) {
          if (rightToggle) rightToggle = !rightToggle;
        }
        // if (rightToggle) {
        // decode.setGate(1.0); // this is 0 or 1
        // } else decode.setGate(0.5);// close gate
        if (dDown()) {
          if (downToggle) downToggle = !downToggle;
        }
        if (downToggle) {
          if (present.Present()) {
            decode.armPosition(1); // launch
          } else decode.armPosition(0); // lift
        }
        telemetry.addData("Top  ball", ball.getTop());
        telemetry.addData("Left ball", ball.getLeft());
        telemetry.addLine(". . . . . . . . . .");
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

  private boolean dDown() {
    boolean done = false;
    while (gamepad2.dpad_down) {
      sleep(2);
      if (gamepad2.dpad_down) done = true;
    }
    return done;
  }

  private boolean dUp() {
    boolean done = false;
    while (gamepad2.dpad_up) {
      sleep(2);
      if (gamepad2.dpad_up) done = true;
    }
    return done;
  }

  private boolean dLeft() {
    boolean done = false;
    while (gamepad2.dpad_left) {
      sleep(2);
      if (gamepad2.dpad_left) done = true;
    }
    return done;
  }

  private boolean dRight() {
    boolean done = false;
    while (gamepad2.dpad_right) {
      sleep(2);
      if (gamepad2.dpad_right) done = true;
    }
    return done;
  }

  private boolean B() {
    boolean done = false;
    while (gamepad1.b) {
      sleep(2);
      if (gamepad1.b) done = true;
    }
    return done;
  }
} // end class Swerve Components
