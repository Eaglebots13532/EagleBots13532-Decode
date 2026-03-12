// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

/* March 7, 2026
 * A test jig is being used to develop the concept of running the decode:hood by encoder
 * The goal is to simplify the hood software and make it more usable.
 * The Hood will have a homing switch to provide an encoder reference. From this reference
 * the hood will move to the desired height using the range to the april tag, provided by
 * the web camera.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drivers.udpwifiData;

@TeleOp(name = "Learning Encoder Basics")
public class LearnEncoderStuff extends LinearOpMode {
  ElapsedTime timeStamp = new ElapsedTime();
  int device = 0;
  udpwifiData snd = new udpwifiData();
  private DcMotorEx motor = null;
  public TouchSensor bob = null;
  int homeEncoder = 0;
  int currentPosition = 0;
  int targetPosition = 0;

  public void runOpMode() {

    motor = hardwareMap.get(DcMotorEx.class, "Motor");
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    sleep(50); // wait for motor driver to process
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    bob = hardwareMap.get(TouchSensor.class, "TouchSensor");

    waitForStart();
    try {
      motor.setPower(0.2);
      timeStamp.reset();
      while (opModeIsActive() && !bob.isPressed()) {
        update();
      }
      motor.setPower(0.0);
      // set encoder count to zero
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      sleep(50); // wait for motor driver to process
      motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      sleep(50);
      timeStamp.reset();
      snd.sendData(0.0, 5, 0.0);
      sleep(1000);
      HoodPosition(-8000);
      HoodPosition(6000);
      HoodPosition(-5000);
      HoodPosition(7000);
      // tells what positions to go to
      // tells what positions to go to

      sleep(1000);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public void HoodPosition(int newPosition) {
    // homeEncoder should = 0 from reset
    if (Math.abs(newPosition) > homeEncoder) {
      targetPosition = newPosition;

      // add a check max position
      int SetTarget = motor.getCurrentPosition() + targetPosition;
      if (SetTarget < -8000) SetTarget = -8000;
      motor.setTargetPosition(SetTarget);
      motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // target position!

      motor.setPower(0.2);
      timeStamp.reset();
      while (opModeIsActive() && motor.isBusy()) {
        update();
      }
    } // end if statement
    motor.setPower(0.0);
  } // end hood position

  void update() {
    try {
      snd.sendData(timeStamp.seconds(), 1, motor.getCurrentPosition());
      snd.sendData(timeStamp.seconds(), 2, motor.getCurrent(CurrentUnit.AMPS));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }
}
