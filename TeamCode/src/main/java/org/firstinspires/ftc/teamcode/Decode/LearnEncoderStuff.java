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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drivers.udpwifiData;

// @TeleOp(name = "Learning Encoder Basics")
@Disabled
public class LearnEncoderStuff extends LinearOpMode {
  private DcMotorEx motor = null;
  public TouchSensor bob = null;
  int homeEncoder = 0;
  int currentPosition = 0;
  int targetPosition = 0;

  udpwifiData sndData = new udpwifiData();

  private final ElapsedTime eTime = new ElapsedTime();

  public void runOpMode() {

    motor = hardwareMap.get(DcMotorEx.class, "Motor");
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    bob = hardwareMap.get(TouchSensor.class, "TouchSensor");

    waitForStart();
    try {

      motor.setPower(0.2);
      while (opModeIsActive() && !bob.isPressed()) {
        // telemetry.addData("Encoder", motor.getCurrentPosition());
        // telemetry.update();
      }
      motor.setPower(0.0);
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      sleep(50);
      motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      HoodPosition(-2000);
      HoodPosition(1000);
      HoodPosition(-3000);
      // tells what positions to go to

    } catch (Exception e) {
      e.printStackTrace();
    }
  } // ends runOpmode

  public void HoodPosition(int newPosition) {
    try {
      if (Math.abs(newPosition) > homeEncoder) {
        targetPosition = newPosition;

        // add a check max position eventually
        targetPosition += motor.getCurrentPosition();
        motor.setTargetPosition(targetPosition);
        sndData.sendData(eTime.milliseconds(), 2, motor.getCurrentPosition());
        sndData.sendData(eTime.milliseconds(), 2, targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // target position!

        motor.setPower(0.2);
        eTime.reset();
        while (opModeIsActive() && motor.isBusy()) {
          if (eTime.seconds() > 1) {
            sndData.sendData(eTime.milliseconds(), 1, motor.getCurrent(CurrentUnit.AMPS));
            eTime.reset();
          }

          telemetry.addData("homeEncoder", homeEncoder);
          telemetry.addData("Encoder", (double) motor.getCurrentPosition());
          telemetry.update();
        }
      } // end if statement
      motor.setPower(0.0);
      sndData.sendData(eTime.milliseconds(), 2, (double) motor.getCurrentPosition());
    } catch (Exception e) {
      e.printStackTrace();
    }
  } // end hood position
}
