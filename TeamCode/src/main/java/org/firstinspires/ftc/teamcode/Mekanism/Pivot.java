package org.firstinspires.ftc.teamcode.Mekanism;

import com.qualcomm.robotcore.hardware.DcMotor;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Pivot extends LinearOpMode {
//  FtcDashboard dashboard = FtcDashboard.getInstance();

  DcMotor motor1;

  @Override
  public void runOpMode() {

    motor1 = hardwareMap.get(DcMotor.class, "MrMotor");

    waitForStart();
    while (opModeIsActive()) {

      motor1.setPower(-gamepad1.left_stick_y);
    }
  }


}// I(Ella) need to change the pivot class to have init and methods to operate pivot

