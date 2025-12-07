// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@TeleOp(name = "TeleOpDecode", group = "Decode")
public class _TankTeleOpDecode extends LinearOpMode {
  DC_Swerve_Drive drive = new DC_Swerve_Drive(this);
  DC_Odometry_Sensor odo = new DC_Odometry_Sensor(this);
  DC_Intake_Launch game = new DC_Intake_Launch(this);
  DC_BallSensor ball = new DC_BallSensor(this);
  // DC_Husky_Sensor ball = new DC_Husky_Sensor("color", this);
  DC_Husky_Sensor tag = new DC_Husky_Sensor("april", this);
  ElapsedTime TimeOut = new ElapsedTime();
  double gpLy = 0.0;
  double gpRy = 0.0;
  double gpLx = 0.0;
  double gpRx = 0.0;
  double gpLt = 0.0;
  double gpRt = 0.0;

  @Override
  public void runOpMode() {
    try {
      // read match side from Autonomous
      String filename = "DecodeMatch.txt";
      // AppUtil.getSettingsFile() places the file in the FIRST/settings folder
      File file = AppUtil.getInstance().getSettingsFile(filename);
      // Use the static method from ReadWriteFile to read the string data
      String match = ReadWriteFile.readFile(file);
      Integer mI = mI = Integer.valueOf(match);
      // This tells DC_Husky_sensor which match side
      tag.side = mI; // This is 1 for Red and 2 for Blue
    } catch (Exception e) {
      telemetry.addData("Read Error", e.getMessage());
      telemetry.update();
    }
    drive.SwerveInit();
    odo.DoInit();
    game.InitIL();
    ball.SensorInit();
    tag.initHuskyLens();

    try {
      game.Intake(); // start intake
      game.armPosition(0); // home arm above ball

      waitForStart();
      while (opModeIsActive()) {
        Drive();
        Decode();
        EndGame();
      } // while op Mode
    } catch (Exception e) {
      telemetry.addLine(", exception in gamePadTeleOP");
      telemetry.update();
      sleep(2000);
      requestOpModeStop();
    } // catch
  } // run Op mode

  public void Drive() {
    gpLt = gamepad1.left_trigger;
    gpLy = gamepad1.left_stick_y;
    gpRt = gamepad1.right_trigger;
    gpRy = gamepad1.right_stick_y;
    // scaleRange(min, max) configure the following servo control by trigger
    // to straf drive to finish alignment to shoot: by April tag
    // check angle of wheel if not 180 then
    gpLt = (1.0 - gamepad1.left_trigger) / 2.0;
    gpRt = 0.5 - (gamepad1.right_trigger) / 2.0;
    if (gpLt < 0.5) {
      // turn left by servo
      drive.lfTurn.setPosition(gpLt);
      drive.rtTurn.setPosition(0.5);
    } else {
      // wheels lock 180/0
      drive.lfTurn.setPosition(0.5);
      drive.rtTurn.setPosition(0.5);
    }
    if (gpRt > 0.5) {
      // turn right by servo
      drive.lfTurn.setPosition(0.5);
      drive.rtTurn.setPosition(gpRt);
    } else {
      // wheels lock 180/0
      drive.lfTurn.setPosition(0.5);
      drive.rtTurn.setPosition(0.5);
    }
    // determine if 90 degrees then turn and lock left
    // determine if -90 degrees then turn and lock right
    // drive wheels
    drive.lfDrive.setPower(gpLy);
    drive.rtDrive.setPower(gpRy);
  } // drive operation

  public void Decode() {
    double rJs = -gamepad2.left_stick_y;
    //
    if (gamepad2.dpad_left) {
      /* Launch requested
      1. check if ball is present
      2. There, close gate
      3. stop intake
      4. set chute they drivers have 1 sec to adjust manually
      5. spin up
      6. wait for launch command
       */
      if (ball.Present()) {
        game.closeGate();
        game.IntakeStop();
        game.chute60(); // default long range 60 deg
        // wait for chute movement or decision for control chute
        sleep(1000);
        if (rJs > 0.1) {
          do {
            rJs = -gamepad2.left_stick_y / 2.0;
            game.chutectl(rJs);
          } while (opModeIsActive() && rJs > .1);
        } // JoyStick control
        TimeOut.reset();
        // change spin-up velocity check range set velocity
        while (opModeIsActive() && game.spinUp(4500.0) && TimeOut.seconds() < 3.0) {
          idle();
        }
        telemetry.addLine("Ready to Launch - dpad_up");
        telemetry.update();
      } // check for ball
    } // prepare to launch ball
    if (gamepad2.dpad_up) {
      /*
      1. launch ball
      2. open gate
      3. start intake
      4. wait for ball to clear
      5. raise arm
      6. spin off
       */
      game.armPosition(1); // lower to launch
      // check for ball present or wait 2 sec
      game.openGate(); // ball away ready for next
      game.Intake(); // start intake
      sleep(1000); // wait for ball to clear or check for ball
      game.armPosition(0);
      game.spinOff(); // raise for next
    } // launch ball
  } // decode

  // drive to position before pushing end game
  public void EndGame() {
    if (gamepad2.x) {
      game.armPosition(3); // lower arm all the way down
      game.chuteHm();
      game.setTilt();
    }
  }
} // tank TeleOp
