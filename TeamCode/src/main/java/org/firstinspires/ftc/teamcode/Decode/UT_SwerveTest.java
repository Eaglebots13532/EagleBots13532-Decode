// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_SwerveTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    var drive = new DC_Swerve_Drive(this);
    drive.init();
    waitForStart();
    while (opModeIsActive()) {
      drive.fieldRelativeDrive(
          -gamepad1.left_stick_y * drive.maxSpeedMetersPerSec,
          -gamepad1.left_stick_x * drive.maxSpeedMetersPerSec,
          -gamepad1.right_stick_x * drive.maxOmegaRadPerSec);
      telemetry.update();
      if (gamepad1.a) {
        drive.resetYaw();
      }
    }
  }
}
