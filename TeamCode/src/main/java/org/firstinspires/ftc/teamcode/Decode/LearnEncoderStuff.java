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
  int targetPosition = 0;

  public void runOpMode() {

    motor = hardwareMap.get(DcMotor.class, "Motor");
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    bob = hardwareMap.get(TouchSensor.class, "TouchSensor");

    waitForStart();

    motor.setPower(0.2);
    while (opModeIsActive() && !bob.isPressed()) {
      telemetry.addData("Encoder", motor.getCurrentPosition());
      telemetry.update();
    }

    motor.setPower(0.0);

    HoodPosition(-2000);
    HoodPosition(-3000);
    HoodPosition(-1500);
    //tells what positions to go to


    sleep(5);
  } // ends runOpMode


  public void HoodPosition(int newPosition){
    if(Math.abs(newPosition) > homeEncoder) {
      targetPosition = newPosition;

      //add a check max position eventually

      motor.setTargetPosition(motor.getCurrentPosition() + targetPosition);
      motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // target position!

      motor.setPower(0.2);
      while (opModeIsActive() && motor.isBusy()) {

        telemetry.addData("homeEncoder", homeEncoder);
        telemetry.addData("Encoder", motor.getCurrentPosition());
        telemetry.update();
      }
    }//end if statement
    motor.setPower(0.0);

  }//end hood position



}
