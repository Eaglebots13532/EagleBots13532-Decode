package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Coles_Robot extends LinearOpMode {

  DcMotor RMotor, LMotor;

  @Override
  public void runOpMode(){

    Init();
    waitForStart();

    while(opModeIsActive()){
      RMotor.setPower(-gamepad1.left_stick_y/2);
      LMotor.setPower(-gamepad1.right_stick_y/2);
    }
  }

  public void Init(){
    RMotor = hardwareMap.get(DcMotor.class, "Rmotor");
    LMotor = hardwareMap.get(DcMotor.class, "Lmotor");

    RMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    RMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    LMotor.setDirection(DcMotorSimple.Direction.REVERSE);
  }
}
