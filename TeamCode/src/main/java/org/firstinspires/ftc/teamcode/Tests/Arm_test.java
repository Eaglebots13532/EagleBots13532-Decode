package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Slide.Arm;

@TeleOp(name = "Arm test")
public class Arm_test extends LinearOpMode {

  public Arm arm;

  @Override
  public void runOpMode() throws InterruptedException {

    Init();

    waitForStart();
    while (opModeIsActive()) {
      arm.get_Arm_Len();
      telemetry.update();
      if(gamepad1.a) {
        arm.offset = arm.arm_Len.getCurrentPosition();
        telemetry.addLine("A button pressed");
        telemetry.addLine("Offset: " + arm.offset);
      }
      arm.set_Arm_Len(-gamepad1.left_stick_y);
      arm.set_Arm_Ang(-gamepad1.right_stick_y);
      //telemetry.addLine();
    }
  }

  public void Init() {
    arm = new Arm(this);
  }
}
