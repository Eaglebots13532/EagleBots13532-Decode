// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utils;

@TeleOp(name = "swerve test")
public class SwerveTest extends LinearOpMode {
  Swerve swerve;
  GoBildaPinpointDriver odo;

  @Override
  public void runOpMode() {

    Init();
    waitForStart();
    int count = 0;

    while (opModeIsActive()) {
      count++;
      double strafe_joystick = gamepad1.left_stick_x;
      double drive_joystick = -gamepad1.left_stick_y;
      double rotate_joystick = gamepad1.right_stick_x;
      swerve.drive(strafe_joystick, drive_joystick, rotate_joystick);
      telemetry.addLine("telemetry updating: " + count);
      telemetry.update();
      odo.update();
    }
  }

  public void Init() {
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odo.recalibrateIMU();
    odo.resetPosAndIMU();
    odo.setOffsets(110, 30);
    odo.setEncoderResolution(goBILDA_4_BAR_POD);
    odo.setEncoderDirections(FORWARD, FORWARD);
//    odo.resetHeading(Rotation2d.fromDegrees(120));
    swerve = new Swerve(this, odo);
  }
}
