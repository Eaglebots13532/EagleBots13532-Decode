// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Learning Encoder Basics")
public class LearnEncoderStuff extends LinearOpMode {
  private DcMotor motor = null;

  public void runOpMode() {
    motor = hardwareMap.get(DcMotor.class, "Motor");

    motor.setPower(0.5);
    sleep(20);

    motor.setPower(0.0);

    telemetry.addData("Encoder", motor.getCurrentPosition());
    telemetry.update();

    sleep(5);
  } // ends runOpMode
}
