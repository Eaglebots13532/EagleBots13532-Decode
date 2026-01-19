// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_Intake_Arm_Chute_Spin_Test extends LinearOpMode {
  private final DC_Intake_Launch game = new DC_Intake_Launch(this);

  @Override
  public void runOpMode() {

    game.InitIL();
    int armEnc = 0;
    boolean leftBump = false;
    boolean rightBump = false;
    int armPos = 0;

    waitForStart();
    // arm positioning
    while (opModeIsActive()) {
      telemetry.addLine(". . . . . . . . . .");
      telemetry.addLine("Right Bumper 2 start 3 sec stop");
      if (gamepad2.right_bumper) {
        game.Intake();
        sleep(3000);
        game.IntakeStop();
      }
      telemetry.addLine(". . . . . . . . . .");
      telemetry.addLine("Left Stick 2 Y");
      telemetry.addData("Arm Encoder Position:", game.arm.getCurrentPosition());
      double armDrv = -gamepad2.left_stick_y / 1.5;
      telemetry.addData("Arm Power:", armDrv);
      game.arm.setPower(armDrv);
      telemetry.addLine(". . . . . . . . . .");
      // chute positioning
      telemetry.addLine("Right Stick 2 Y");
      telemetry.addData("Chute potentiometer Position:", game.chuteVal.getVoltage());
      double chuteDrv = -gamepad2.right_stick_y / 2.0;
      telemetry.addData("Chute Power:", chuteDrv);
      game.chute.setPower(chuteDrv);
      telemetry.addLine(". . . . . . . . . .");
      // spin velocity
      telemetry.addLine("Left Trigger 2 Y");
      telemetry.addData("Spin Velocity:", game.launch.getVelocity());
      int spinVeloc = (int) (gamepad2.left_trigger * 6000);
      telemetry.addData("request Velocity:", spinVeloc);
      game.launch.setVelocity(spinVeloc);
      telemetry.addLine(". . . . . . . . . .");
      telemetry.update();
    }
  } // run OpMode
}
