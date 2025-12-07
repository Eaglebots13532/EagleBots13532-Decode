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
drag1  Servo  4 . . . . . . . . . . drag1
drag2  Servo  5 . . . . . . . . . . drag2
Intake Servo  2 . . . . . . . . . . intake
Guide  Servo  3 . . . . . . . . . . gate

Ball   Rev Color Sensor . . . IC0 1 ballSensor
Ball 1 ???
Ball 2 ???
April Tag  . . . . . . . . . .IC1 0 april
Husky Camera   . . . . . . . .IC2 0 huskyLens
PinPoint . . . . . . . . . . .IC3 0 odo

 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_Swerve_tests extends LinearOpMode {

  // Swerve Devices
  // -----------------------------
  DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  DC_Odometry_Sensor ppo = new DC_Odometry_Sensor(this);

  // Game Pad controls
  double gpY = 0.0;
  double gpX = 0.0;
  double limitSpeed = 0.8;
  double motorSpeed = 0.0;
  final double turnslope = 0.5315;
  final double zeroOffset = 0.85;
  final double spv180 = 2.52;
  final double servoMaxV = 3.03;
  double startSv = 0.0;
  double endSv = 0.0;
  double turnVolt = spv180; // radians
  double wheelTurn = 0.5; // centered 180
  double setServo = turnVolt * turnslope + zeroOffset;
  // toggles
  boolean upToggle = false;
  boolean btoggle = false;

  @Override
  public void runOpMode() {
    // Initialize class components
    drive.SwerveInit();
    ppo.DoInit();
    // Wait for the DS start button to be touched.
    telemetry.addLine("- - Game Pad A - -");
    telemetry.addLine("left stick Y drive");
    telemetry.addLine("right stick X turn");

    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      boolean setServoDir = true;
      try {
        // x & y are the drive position
        gpY = -gamepad1.left_stick_y;
        gpX = gamepad1.right_stick_x;

        wheelTurn = (gpX + 1.0) / 2.0;
        // map joystick 0 to 1 as 90 to 180 and | -1 to 0 | 180 to 270 return sign to motor drive
        turnVolt = servoMaxV + (zeroOffset - servoMaxV) * gpX; // seek potentiometer voltage
        motorSpeed = gpY * limitSpeed;
        // turning 180 - 270 causes the motor to reverse
        if (motorSpeed > 1.0) motorSpeed = 1.0;
        if (motorSpeed < -1.0) motorSpeed = -1.0;

        drive.lfTurn.setPosition(wheelTurn);
        drive.rtTurn.setPosition(wheelTurn);
        // if(Math.abs(gpX) > .1)motorSpeed *= Math.signum(gpX);
        drive.lfDrive.setPower(motorSpeed); // turn on motor drive
        drive.rtDrive.setPower(motorSpeed);
        //
        // telemetry.addData("gpX ",gpX);
        // telemetry.addData("gpY ",gpY);
        telemetry.addData("Est. radian", (turnVolt - zeroOffset) / turnslope);
        telemetry.addData("LSpv  radian", (drive.lfsa - zeroOffset) / turnslope);
        telemetry.addData("RSpv  radian", (drive.rtsa - zeroOffset) / turnslope);
        telemetry.addLine("Servo request");
        telemetry.addData("Left ", drive.lfTurn.getPosition());
        telemetry.addData("Right", drive.rtTurn.getPosition());
        telemetry.addLine(". . . . . . . . . .");
        telemetry.addLine("Servo potentiometer");
        telemetry.addData("Left ", drive.lfSPot.getVoltage());
        telemetry.addData("Right", drive.rtSPot.getVoltage());
        telemetry.addLine(". . . . . . . . . .");
        telemetry.addLine("Drive Motor velocity");
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Left ", drive.lfDrive.getVelocity());
        telemetry.addData("Right", drive.rtDrive.getVelocity());
        telemetry.addLine("Odometry values");
        telemetry.addData("X:", ppo.getXPosition());
        telemetry.addData("Y:", ppo.getYPosition());
        telemetry.addData("B:", ppo.getBearing());
        if (btoggle) telemetry.addLine("Speed Limit .8");
        else telemetry.addLine("Speed Limit .5");
        telemetry.update();

      } // end try
      catch (Exception e) {
        telemetry.addLine(", exception in UT swerve test");
        telemetry.update();
        sleep(2000);
        requestOpModeStop();
      } // catch exception
    } // while op mode running
  } // run op mode

  private boolean dDown() {
    boolean done = gamepad2.dpad_down;
    return done;
  }

  private boolean dUp() {
    boolean done = gamepad2.dpad_up;
    return done;
  }

  private boolean dLeft() {
    boolean done = gamepad2.dpad_left;
    return done;
  }

  private boolean dRight() {
    boolean done = gamepad2.dpad_right;
    return done;
  }

  private boolean B() {
    boolean done = gamepad1.b;
    return done;
  }
} // end class Swerve Components
