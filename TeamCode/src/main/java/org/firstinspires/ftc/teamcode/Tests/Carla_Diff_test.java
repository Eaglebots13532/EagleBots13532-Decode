package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mekanism.Carla_Differential;

@TeleOp (name = "Carla's differential test")
public class Carla_Diff_test extends LinearOpMode {

  Carla_Differential diff;

  @Override
  public void runOpMode(){

    double pos;
    double rotation;
    Init();
    waitForStart();
    while(opModeIsActive()){
      pos = -gamepad1.left_stick_y;
      rotation = gamepad1.right_stick_x;
      diff.move(pos,rotation);
      telemetry.addLine("left diff pow: " + diff.left.getPower());
      telemetry.addLine("right diff pow: " + diff.right.getPower());
      telemetry.addLine("input pos: " + pos);
      telemetry.update();
    }
  }

  public void Init(){
    diff = new Carla_Differential(this);
  }
}
