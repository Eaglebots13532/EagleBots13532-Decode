// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestServos extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    var leftServo = hardwareMap.servo.get("LFS");
    var rightServo = hardwareMap.servo.get("RFS");
    var leftEncoder = hardwareMap.analogInput.get("LFP");
    var rightEncoder = hardwareMap.analogInput.get("RFP");
    waitForStart();
    while (opModeIsActive()) {
      leftServo.setPosition(.5);
      rightServo.setPosition(.5);
      telemetry.addData("Left Encoder Voltage", leftEncoder.getVoltage());
      telemetry.addData("Right Encoder Voltage", rightEncoder.getVoltage());
      telemetry.update();
    }
  }
}
