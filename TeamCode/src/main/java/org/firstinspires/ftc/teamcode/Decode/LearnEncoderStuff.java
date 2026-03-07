// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Learning Encoder Basics")
public class LearnEncoderStuff extends LinearOpMode {
  private DcMotor motor = null;
  public TouchSensor bob = null;
  int homeEncoder = 0;

  int currentPosition = 0;

  public void runOpMode() {
    motor = hardwareMap.get(DcMotor.class, "Motor");
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    bob = hardwareMap.get(TouchSensor.class, "TouchSensor");

    waitForStart();

    motor.setPower(0.3);
    while (opModeIsActive() && !bob.isPressed()) {
      telemetry.addData("Encoder", motor.getCurrentPosition());
      telemetry.update();
    }

    motor.setPower(0.0);

    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    motor.setPower(-0.5);
    while (opModeIsActive() && Math.abs(motor.getCurrentPosition()) < homeEncoder + 5000) {

      telemetry.addData("homeEncoder", homeEncoder);
      telemetry.addData("Encoder", motor.getCurrentPosition());
      telemetry.update();
    }

    motor.setPower(0.0);

    currentPosition = motor.getCurrentPosition();

    motor.setPower(0.5);
    while (opModeIsActive() && Math.abs(motor.getCurrentPosition()) > currentPosition - 500) {

      telemetry.addData("homeEncoder", homeEncoder);
      telemetry.addData("Encoder", motor.getCurrentPosition());
      telemetry.update();
    }
    motor.setPower((0.0));

    sleep(5);
  } // ends runOpMode
}
