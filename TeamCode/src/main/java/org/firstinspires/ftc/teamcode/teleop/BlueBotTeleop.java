// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.TheBestSwerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;

@Config
@TeleOp(name = "Blue Bot Teleop")
public class BlueBotTeleop extends LinearOpMode {

  FtcDashboard dash = FtcDashboard.getInstance();

  double slideSpeed = 80;
  GoBildaPinpointDriver odometry;

  boolean
      is2A = false,
      is2B = false,
      is2X = false,
      is2Y = false,
      game2A = false,
      game2B = false,
      game2X = false,
      game2Y = false;

  public final double change_In_Offset = .025;

  @Override
  public void runOpMode() throws InterruptedException {

    //if you want outputs on driver station comment out the line below
    telemetry = dash.getTelemetry();

    // Does this move the robot? not anymore but you need to init the wrist or press b to get it to go to the right position
    Mekanism mek = new Mekanism(this);

    Init();
    waitForStart();
    mek.arm.homeArm();
    mek.grabber.initWrist();
    odometry.resetHeading();
    odometry.resetPosAndIMU();

    mek.arm.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.RunMode.RUN_USING_ENCODER);

    while (opModeIsActive()) {

      double left_joy_x = gamepad1.left_stick_x;
      double left_joy_y = gamepad1.left_stick_y;

      /**
       * To this point, the code supports strafe. Steering can be determined by taking into
       * consideration the drive speed and the value of the right joystick x-axis.
       * Multiplying by -1 is because smaller numbers of steering_angle are on the right
       * side whereas -1 is on the left side of the x-axis joystick.
       */
      double right_joy_x = gamepad1.right_stick_x * -1.0;

      amazingSwerve.swerveTheThing(left_joy_x, left_joy_y, right_joy_x);

      //just in case auto get's screwed up
      if (gamepad1.b && gamepad1.a) {
        amazingSwerve.odometry.resetHeading(new Rotation2d());
        sleep(250);
      }

      /*
        Everything before this is for Driving.
        Everything below is for Mekanism
       */

      double
          g2_lx = gamepad2.left_stick_x,
          g2_ly = -gamepad2.left_stick_y,
          g2_rx = gamepad2.right_stick_x,
          g2_ry = -gamepad2.right_stick_y,
          g2_lt = gamepad2.left_trigger,
          g2_rt = gamepad2.right_trigger;

      telemetry.addLine("G2 LY: " + g2_ly);
      telemetry.addLine("G2 RY: " + g2_ry);
      if (gamepad2.right_bumper) {
        mek.arm.hang();
        mek.grabber.setGrabber(0, 0);
        sleep(5000);
      } else {
        mek.arm.setSlide(g2_ly);
        mek.arm.setPivot(g2_ry);
      }
      telemetry.addLine("Slide current pos: " + mek.arm.slide.getPower());
      telemetry.addLine("Slider current pos: " + mek.arm.slide.getCurrentPosition() + "Slide goal pos: " + mek.arm.slide.getTargetPosition());

      // This block handles making the gamepad.b toggle the wrist position
      if (gamepad2.b && !is2B) {
        game2B = !game2B;
        is2B = true;
      } else if (!gamepad2.b) is2B = false;
      if (game2B)
        mek.grabber.setWrist(1.0);
      else mek.grabber.setWrist(-1.0);

      // Grabber power
      double grabberSpeed = g2_rt - g2_lt;
      mek.grabber.setGrabber(grabberSpeed, grabberSpeed);
      if (gamepad2.a && gamepad2.b) {
        mek.arm.slide.setMode(STOP_AND_RESET_ENCODER);
        mek.arm.slide2.setMode(STOP_AND_RESET_ENCODER);
        mek.arm.slide.setMode(RUN_USING_ENCODER);
        mek.arm.slide2.setMode(RUN_USING_ENCODER);
      }
      mek.update();
      telemetry.addLine("Grabber speed: " + grabberSpeed);
      telemetry.addLine("intake 1 power: " + mek.grabber.intake1.getPosition());
      telemetry.addLine("intake 2 power: " + mek.grabber.intake2.getPosition());

      telemetry.addLine("Imu angle: " + odometry.getHeading().getDegrees());
      telemetry.addLine("X Pos: " + odometry.getPosX());
      telemetry.addLine("Y Pos: " + odometry.getPosY());
      telemetry.update();
    }
  }

  public void Init() {
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odometry.recalibrateIMU();
    odometry.resetPosAndIMU();
    odometry.setOffsets(160, 32.5);
    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
    odometry.setEncoderDirections(FORWARD, FORWARD);
    odometry.resetHeading(Rotation2d.fromDegrees(120));
  }
}
