// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.REVERSED;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.TheBestSwerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utils;


@Autonomous(name = "Auto Top Bucket", preselectTeleOp = "Blue Bot Teleop")
public class AutoBucket extends LinearOpMode {
  private AutoSwerve driveBase;
  private GoBildaPinpointDriver odo;
  private Mekanism mek;
  TheBestSwerve amazingSwerve;
  private double perfect_voltage;
  private double power_volt;

  private void stop_movement() {
    amazingSwerve.swerveTheThing(0.0, 0.0, 0.0);
    sleepWithAmazingSwerve(100);
  }

  private void move_robot(double left_x, double left_y, double right_x, int sleep_ms) {
    left_x *= power_volt;
    left_y *= power_volt;
    right_x *= power_volt;
    amazingSwerve.swerveTheThing(left_x, left_y, right_x);
    sleepWithAmazingSwerve(sleep_ms);
    stop_movement();
  }

  @Override
  public void runOpMode() throws InterruptedException {
    // two adjustable values are
    VoltageSensor voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    double initialVoltage = voltageSensor.getVoltage();
    perfect_voltage = 13.5;
    power_volt = perfect_voltage / initialVoltage;
    initOdo();
    driveBase = new AutoSwerve(this, odo);
    mek = new Mekanism(this);
    amazingSwerve = new TheBestSwerve(this, odo, driveBase);

    waitForStart();
    mek.arm.homeArm();
    mek.grabber.initWrist();
    mek.grabber.setWrist(1.0);
    mek.grabber.setGrabber(0, 0);
    mek.update();

    // raise arm to top bucket
    topBucket();
    sleep(100);
    //secondTopBucket();
    telemetry.update();
    telemetry.addLine("top bucket done");
    move_robot(0.0, 0.0, 0.7, 900);
    sleep(2000);

//    telemetry.addLine("into corner");
//    move_robot(0.8, 0.0, 0.0, 3000);

    odo.update();
    telemetry.update();

//    amazingSwerve.swerveTheThing(0,0,-0.1);
//    sleepWithAmazingSwerve(250);
    while (opModeIsActive()) ;
  }// end runOpMode

  public void secondTopBucket() {
    telemetry.addLine("out");
    move_robot(0.55, 0.0, 0.0, 1700);

    telemetry.addLine("Rotate to pick up");
    move_robot(0.0, 0.0, -.545, 1300);
    //sleep(20000);

    mek.arm.setSlide(1350);
    mek.grabber.setGrabber(-1, -1);
    sleepWithMekUpdate(750);
    telemetry.addLine("pivot down");
    mek.arm.setPivot(86);
    sleepWithMekUpdate(1850);
    mek.arm.setSlide(1450);
    sleepWithMekUpdate(100);

    telemetry.addLine("pivot up");
    mek.arm.setPivot(0);
    sleepWithMekUpdate(2000);
    telemetry.addLine("mek angle: " + mek.arm.pivot.getCurrentPosition());
    sleep(1500);

    telemetry.addLine("Rotate to drop");
    move_robot(0.0, 0.0, 0.55, 2100);
    sleep(50);
    telemetry.addLine("top bucket");
    mek.arm.setSlide(4100);
    mek.arm.setPivot(0);
    sleepWithMekUpdate(2250);
    mek.arm.setPivot(13);
    sleepWithMekUpdate(750);
    mek.grabber.setGrabber(0.75, 0.5);
    sleepWithMekUpdate(1250);
    mek.grabber.setGrabber(0, 0);
    mek.update();
    mek.arm.setPivot(0);
    sleepWithMekUpdate(500);
    mek.arm.setSlide(0);
    sleepWithMekUpdate(1750);

    //about 3 seconds remaining here ----------------------------------------- //

    telemetry.addLine("Rotate back out to up");
    move_robot(0.0, 0.0, -.54, 1700);
    telemetry.addLine("back");
    move_robot(-0.2, .8, 0.0, 1500);
//    telemetry.addLine("into corner");
//    move_robot(0.8, 0.0, 0.0, 3000);
  }

  public void topBucket() {
    telemetry.addLine("slide out");
    mek.arm.setSlide(4100);
    sleepWithMekUpdate(2250);
    mek.arm.setPivot(15);
    sleepWithMekUpdate(750);
    mek.grabber.setGrabber(.75, .5);
    sleepWithMekUpdate(1250);
    mek.grabber.setGrabber(0, 0);
    mek.arm.setPivot(0);
    sleepWithMekUpdate(750);
    mek.arm.setSlide(0);
    sleepWithMekUpdate(1500);
  }

  public void sleepWithMekUpdate(int timeInMS) {
    double currentTime = Utils.getTimeMiliSeconds();
    double endTime = Utils.getTimeMiliSeconds() + timeInMS;
    while (currentTime < endTime && opModeIsActive()) {
      mek.update();
      mek.arm.update();
      mek.grabber.update();
      currentTime = Utils.getTimeMiliSeconds();
      try {
        telemetry.addData("Slide pos: ", mek.arm.slide.getCurrentPosition());
        telemetry.addData("Pivot pos: ", mek.arm.pivot.getCurrentPosition());
        telemetry.addLine("grabber1 power: " + mek.grabber.intake1.getPosition());
        telemetry.addLine("grabber2 power: " + mek.grabber.intake2.getPosition());
        telemetry.addLine("pivot target pos: " + mek.arm.pivot.getTargetPosition());
        telemetry.addLine("pivot current pos: " + mek.arm.pivot.getCurrentPosition());
        telemetry.addLine("pivot power: " + mek.arm.pivot.getPower());
      } catch (Exception e) {
        telemetry.addLine("error");
      }
      telemetry.update();
    }
  }

  public void sleepWithAmazingSwerve(double timeInMS) {
    double currentTime = Utils.getTimeMiliSeconds();
    double endTime = Utils.getTimeMiliSeconds() + timeInMS;
    while (currentTime < endTime && opModeIsActive()) {
      odo.update();
      currentTime = Utils.getTimeMiliSeconds();
    }
  }

  public void initOdo() {
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odo.resetPosAndIMU();
    sleep(250);
    odo.setOffsets(100, 90);
    sleep(100);
    odo.setEncoderResolution(goBILDA_4_BAR_POD);
    odo.setEncoderDirections(FORWARD, REVERSED);
    odo.resetHeading(Rotation2d.fromRadians(1.88));
  }

  public void drive(double x, double y, double heading) {
    double heading2 = Math.atan2(x, y);
    if (heading2 < 0)
      heading2 += 360;
    driveBase.set_wheels(heading2, heading2, heading2, heading2, odo.getHeading().getRadians());
  }

  public void moveRobot(double change_x, double change_y, double steer_amt) {
    double current_x = odo.getPosX();
    double current_y = odo.getPosY();
//    double current_h = odo.getHeading().getRadians();
    double wanted_x = current_x + change_x;
    double wanted_y = current_y + change_y;
    double accuracy = 0.02;
//    double wanted_h = current_h - steer_amt;

    while ((!(change_x < accuracy && change_x > -accuracy) || !(change_y < accuracy && change_y > -accuracy)) && opModeIsActive()) {

      //reduce joystick input for reduction of motor speed
      double limit = .60;
      if (change_x > accuracy && change_x > limit) change_x = limit;
      if (change_x < -accuracy && change_x < -limit) change_x = -limit;
      if (change_y > accuracy && change_y > limit) change_y = limit;
      if (change_y < -accuracy && change_y < -limit) change_y = -limit;
      telemetry.addLine("" + change_y);

      limit = .45; // not sure if it should be same as above
      if (steer_amt > accuracy && steer_amt > limit) steer_amt = limit;
      if (steer_amt < -accuracy && steer_amt < -limit) steer_amt = -limit;


      //increase joystick input so that motors actually move
      double min = 0.35;
      if (change_x > accuracy && change_x < min) change_x = min;
      else if (change_x > -min && change_x < -accuracy) change_x = -min;
      else if (change_x > -accuracy && change_x < accuracy) change_x = 0;
      if (change_y > accuracy && change_y < min) change_y = min;
      else if (change_y > -min && change_y < -accuracy) change_y = -min;
      else if (change_y > -accuracy && change_y < accuracy) change_y = 0;
      telemetry.addLine("" + change_y);

      min = 0.15; //idk if it should be the same as the ones above
      if (steer_amt < min && steer_amt > accuracy) steer_amt = min;
      if (steer_amt > -min && steer_amt < accuracy) steer_amt = -min;
      else steer_amt = 0;

      //actually move robot
      amazingSwerve.swerveTheThing(change_x, change_y, 0.0);

      //update variables "current" and "change"
      current_x = odo.getPosX();
      current_y = odo.getPosY();
      change_x = wanted_x - current_x;
      change_y = wanted_y - current_y;
      change_x *= -1;
      change_y *= -1;

      //to try to compensate for stupid stuff
//      if(wanted_y < -current_y)
//        change_y *= -1;

      //print
      outputPosition();
      telemetry.addLine("wanted pos x: " + wanted_x);
      telemetry.addLine("wanted pos y: " + wanted_y);
      telemetry.addLine("change x (w - c): " + change_x);
      telemetry.addLine("change y (w - c): " + change_y);
      telemetry.update();
      odo.update();
    }
  }

  public void outputPosition() {
    telemetry.addLine("x pos: " + odo.getPosX());
    telemetry.addLine("y pos: " + odo.getPosY());
    telemetry.addLine("heading (degrees): " + odo.getHeading().getDegrees());
  }

  /**
   * targetPos Pose2d target position relative to the field. 0,0 is where the robot first started
   * timeLimit Time to take to move the specified distance
   * needs to be entirely rewritten
   */
  /*
  public void driveWithOdo(Pose2d targetPos, double timeLimit) {

    ElapsedTime timer = new ElapsedTime();
    timer.reset();

    Pose2d currentPos;

    double
        integralX = 0,
        integralY = 0,
        integralHeading = 0,
        previousErrorX = 0,
        previousErrorY = 0,
        previousErrorHeading = 0,
        lastTime = 0;

    while (timer.seconds() < timeLimit && opModeIsActive()) {

      // 1. Get the current position of the robot
      currentPos = drivebase.getPose();

      // 2. Calculate the error in position
      double errorX = targetPos.getX() - currentPos.getX();
      double errorY = targetPos.getY() - currentPos.getY();
      double errorHeading = targetPos.getRotation().getRadians() - currentPos.getRotation().getRadians();

      // 3. Calculate the time since the last loop through
      double currentTime = timer.startTime();
      double deltaTime = currentTime - lastTime;

      // 4. Calculate the PID outputs
      double pidOutputX = calculatePID(errorX, deltaTime, 0.05, 0.0, 0.0, integralX, previousErrorX);
      double pidOutputY = calculatePID(errorY, deltaTime, 0.05, 0.0, 0.0, integralY, previousErrorY);
      double pidOutputHeading = calculatePID(errorHeading, deltaTime, 0.05, 0.0, 0.0, integralHeading, previousErrorHeading);

      // 5. Update the integral and previous error
      integralX += errorX * deltaTime;
      integralY += errorY * deltaTime;
      integralHeading += errorHeading * deltaTime;
      previousErrorX = errorX;
      previousErrorY = errorY;
      previousErrorHeading = errorHeading;

      // 6. Create speeds
      ChassisSpeeds speeds = new ChassisSpeeds(pidOutputX, pidOutputY, pidOutputHeading);

      // 7. Update speeds for the robot
      drivebase.drive(speeds, deltaTime);
      /*
      // 8. Telemetry - Telemetry causes a null exception for some reason
      telemetry.addData("Current X",0);
      telemetry.addData("Current Y", 0);
      telemetry.addData("Current Heading", 0);
      telemetry.addData("Time Remaining", 0);
      telemetry.update();

      // 9. Check if its close enough to the target
      if (Math.abs(errorX) < 0.5 && Math.abs(errorY) < 0.5 && Math.abs(errorHeading) < Math.toRadians(5)) {
        break;
      }

      if (timer.seconds() > timeLimit) {
        break;
      }
    }

    // Stops the robot
    drivebase.drive(new ChassisSpeeds(0, 0, 0), 0);
  }
  */
  private double calculatePID(double error, double deltaTime, double kP, double kI, double kD, double integral, double previousError) {
    double derivative = (error - previousError) / deltaTime;
    return kP * error + kI * integral + kD * derivative;
  }
}
