// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UT_Arm_Chute_Spin_Test extends LinearOpMode {
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
      telemetry.addData("Arm Encoder Position:", game.arm.getCurrentPosition());
      double armDrv = -gamepad1.left_stick_y / 1.5;
      telemetry.addData("Arm Power:", armDrv);
      game.arm.setPower(armDrv);
      telemetry.addLine(". . . . . . . . . .");
      // chute positioning
      telemetry.addData("Arm potentiometer Position:", game.chuteVal.getVoltage());
      double chuteDrv = -gamepad1.right_stick_y / 2.0;
      telemetry.addData("Chute Power:", chuteDrv);
      game.chute.setPower(chuteDrv);
      telemetry.addLine(". . . . . . . . . .");
      // spin velocity
      telemetry.addData("Spin Velocity:", game.launch.getVelocity());
      int spinVeloc = (int) (-gamepad2.left_stick_y * 6000);
      telemetry.addData("request Velocity:", spinVeloc);
      game.launch.setVelocity(spinVeloc);
      telemetry.addLine(". . . . . . . . . .");
      telemetry.update();
    }
  } // run OpMode
}
