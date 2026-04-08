// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.cole;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank Drive With Triggers", group = "TeleOp")
public class Cole extends LinearOpMode {

  private DcMotor leftFront;
  private DcMotor leftRear;
  private DcMotor rightFront;
  private DcMotor rightRear;
  private CRServo arm;

  public void runOpMode() {

    // Hardware mapping
    arm = hardwareMap.get(CRServo.class, "arm");
    leftFront = hardwareMap.get(DcMotor.class, "LFMotor");
    leftRear = hardwareMap.get(DcMotor.class, "LRMotor");
    rightFront = hardwareMap.get(DcMotor.class, "RFMotor");
    rightRear = hardwareMap.get(DcMotor.class, "RRMotor");

    // Reverse left side if needed
    leftFront.setDirection(DcMotor.Direction.REVERSE);
    rightRear.setDirection(DcMotor.Direction.REVERSE);

    waitForStart();

    while (opModeIsActive()) {

      // Stick control (Tank Drive)
      double leftPower = -gamepad1.left_stick_y;
      double rightPower = -gamepad1.right_stick_y;

      // Trigger override
      double forward = gamepad1.right_trigger;
      double backward = gamepad1.left_trigger;

      if (forward > 0.05) {
        leftPower = forward;
        rightPower = forward;
      } else if (backward > 0.05) {
        leftPower = -backward;
        rightPower = -backward;
      }

      // Set Servo Power using Dpad
      if (gamepad1.dpad_left) {
        arm.setPower(-1);
      } else if (gamepad1.dpad_right) {
        arm.setPower(1);
      }

      // Set motor power
      leftFront.setPower(leftPower);
      leftRear.setPower(leftPower);
      rightFront.setPower(rightPower);
      rightRear.setPower(rightPower);

      telemetry.addData("Left Power", leftPower);
      telemetry.addData("Right Power", rightPower);
      telemetry.update(); //
    }
  }
}
