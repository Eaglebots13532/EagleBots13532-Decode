// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_Intake_launch_Test extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);
  private final DC_BallSensor Ball = new DC_BallSensor(this);
  private final DC_Swerve_Drive Drv = new DC_Swerve_Drive(this);
  private final DC_Husky_Sensor ball = new DC_Husky_Sensor("color", this);
  private final DC_Husky_Sensor tag = new DC_Husky_Sensor("april", this);

  @Override
  public void runOpMode() {

    game.InitIL();
    Ball.SensorInit();

    waitForStart();

    if (opModeIsActive()) {
      // start intake system
      double moveLR = 0.0;
      double moveFB = 0.0;
      game.Intake();

      sleep(500);

      sleep(500);
      game.launchVelocity();
      while (opModeIsActive() && Ball.Present()) {
        // GId ball found
        if (ball.GId() > 0) {
          moveLR = ball.getLeft(); // may need neg. of
          moveFB = ball.getTop() - 1.0; // move back picking ball
        }
      }
      // spin up
      while (opModeIsActive() && game.spinUp(200)) {
        idle();
      }
      sleep(1000);
      game.Intake();
      game.spinOff();
      //
      game.Intake();
      while (opModeIsActive() && game.spinUp(400)) {
        idle();
      }
      sleep(3000);

      game.IntakeStop();
      game.spinOff();
    }
  }
}
