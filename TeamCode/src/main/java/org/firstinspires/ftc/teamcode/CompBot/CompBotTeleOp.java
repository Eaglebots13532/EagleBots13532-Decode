// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.CompBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mekanism.*;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;

@Disabled
@TeleOp(name = "CompBot TeleOp Swerve", group = "CompBot")
public class CompBotTeleOp extends LinearOpMode {

//  Swerve swerve = new Swerve(this);
  Mekanism mek = new Mekanism(this);

  double wristAngle = 0, armX = 0, armY = 0;
  ElapsedTime wristTimer1 = new ElapsedTime();
  ElapsedTime wristTimer2 = new ElapsedTime();
  ElapsedTime armXTimer = new ElapsedTime();
  ElapsedTime armYTimer = new ElapsedTime();

  /**
   * Controls for Gamepad 1: Right trigger: Forwards Left trigger: Reverse Right stick X: Rotate
   * Left stick X Strafe
   *
   * <p>Controls for Gamepad 2: Left stick y: In and out of arm Right stick y: Up and down of arm
   * Left trigger: Claw intake Right trigger: Claw out Presets for: Attaching clip to sample
   * Attaching specimen(clip + sample) to top rung Presets for bucket 1 and 2
   */
  public void runOpMode() throws InterruptedException {

    waitForStart();
    while (opModeIsActive()) {

//      swerve.teleopDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//      swerve.periodic();
      telemetry.update();

      // Wrist one way
      if (gamepad2.dpad_up && wristTimer1.milliseconds() > 5 && opModeIsActive()) {
        wristAngle += 0.1;
      } else wristTimer1.reset();

      // Wrist the other way
      if (gamepad2.dpad_down && wristTimer2.milliseconds() > 5 && opModeIsActive()) {
        wristAngle -= 0.1;
      } else wristTimer2.reset();

      // Arm X length
      if (gamepad2.left_stick_x != 0 && armXTimer.milliseconds() > 5 && opModeIsActive()) {
        armX += Math.pow(gamepad2.left_stick_x, 3);
      } else armYTimer.reset();

      // Arm Y length
      if (gamepad2.left_stick_y != 0 && armYTimer.milliseconds() > 5 && opModeIsActive()) {
        armY += -Math.pow(gamepad2.left_stick_y, 3);
      } else armYTimer.reset();

//      mek.wristAngle(wristAngle);
//      mek.moveXY(armX, armY);

      // If any input related to the mekanism is moved, cancel the auto movement
      if (gamepad2.dpad_up
          || gamepad2.dpad_down
          || gamepad2.left_stick_x != 0
          || gamepad2.left_stick_y != 0) {
//        mek.autoClipRun.set(false);
      }
    }
  }
}
