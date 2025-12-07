// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "_Comp Tank Drive")
public class Comp_TankDrive extends LinearOpMode {

  DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
  DC_Intake_Launch game = new DC_Intake_Launch(this);

  @Override
  public void runOpMode() throws InterruptedException {
    drive.SwerveInit();
    odo.DoInit();
    game.InitIL();

    waitForStart();

    int count = 0;
    while (opModeIsActive()) {
      telemetry.addLine("Driving wheels");
      drive.lfDrive.setPower(-gamepad1.left_stick_y);
      drive.rtDrive.setPower(-gamepad1.right_stick_y);

      // chute setting
      telemetry.addLine("Setting chute");
      game.chute.setPower(-gamepad2.left_stick_y);
      // arm setting
      telemetry.addLine("Setting arm");
      game.arm.setPower(-gamepad2.right_stick_y);
      // launcher setting
      telemetry.addLine("Setting shooter");
      game.launch.setPower(gamepad2.right_stick_x);
      // intake setting
      telemetry.addLine("Setting intake");
      game.intake.setPower(gamepad2.left_stick_x);

      // telemetry update for running
      telemetry.addLine("Count: " + count);
      telemetry.update();
    }
  }
}
