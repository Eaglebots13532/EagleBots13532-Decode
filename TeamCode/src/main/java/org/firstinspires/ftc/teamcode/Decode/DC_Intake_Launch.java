// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DC_Intake_Launch {
  /* Declare OpMode members.
   * gain access to methods in the calling OpMode.
   */
  private LinearOpMode myOp = null;

  // Default constructor
  public DC_Intake_Launch(LinearOpMode opmode) {
    myOp = opmode;
    // present = new DC_BallSensor(myOp);
  }

  // public DC_BallSensor present;

  public DcMotorEx launch = null; // 6000 rpm motor
  public DcMotor arm = null; // 312 rpm motor

  public CRServo intake = null; // intake motor controller
  public Servo gate = null; // intake gate
  public Servo tilt = null; // tilt robot up

  // time out timer
  private ElapsedTime runTime = new ElapsedTime();
  // global variables
  public int encHome = 0;

  // status light

  public void InitIL() {

    // Define and Initialize Motor.
    launch = myOp.hardwareMap.get(DcMotorEx.class, "launch");
    launch.setDirection(DcMotorSimple.Direction.FORWARD); // todo set launch direction
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // prepare use velocity
    arm = myOp.hardwareMap.get(DcMotor.class, "arm");
    arm.setDirection(DcMotorSimple.Direction.FORWARD); // todo set arm direction

    // Define and Initialize Servo
    intake = myOp.hardwareMap.get(CRServo.class, "intake");
    // gate to stop balls from entering the chute
    gate = myOp.hardwareMap.get(Servo.class, "gate"); // todo set gate position
    tilt = myOp.hardwareMap.get(Servo.class, "tilt");
    // present.SensorInit();
    encHome = arm.getCurrentPosition(); // arm starts in home position
  }

  // Servo controlled motor
  public void Intake() {
    intake.setPower(-1.0); // front intake servo motor
  }

  public void IntakeStop() {
    intake.setPower(0.0); // front intake servo motor
  }

  // end motor checks

  public boolean spinUp(double shoot_velocity) {
    boolean ready = false; // the not of false
    launch.setVelocity(shoot_velocity);
    myOp.sleep(100);
    double presVelocity = launch.getVelocity();
    // check for ball causing stall
    if (presVelocity < 100) {
      spinOff();
      myOp.sleep(50);
      // reverse fly wheel kicking ball back and start again
      launch.setVelocity(-500);
      myOp.sleep(100);
      launch.setVelocity(shoot_velocity);
    }
    runTime.reset();
    // spin up fly wheel checking for reached velocity false time out
    while (myOp.opModeIsActive() && runTime.seconds() < 5.0 && !ready) {
      if (launch.getVelocity() > shoot_velocity) ready = true;
    }
    // set status light
    return ready;
  } // end spin up

  public void spinOff() {
    launch.setVelocity(0.0);
  }

  /*
   launch arm has three positions
   home    above ball
   launch  position to launch ball
   endGame end game retraction

   Autonomous must start at home positon above ball
   arm position is movement is from the home start position
   at the end of autonomous the launch arm should be set to home
  */
  public boolean armPosition(int pos) {
    double armPwr = 0.3;

    final int home = encHome;
    final int endGame = encHome + 20;

    int move = 0;
    if (pos == 0) move = home;
    else if (pos == 2) move = endGame;
    arm.setTargetPosition(move);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set the run mode
    arm.setPower(armPwr);
    runTime.reset();
    while (myOp.opModeIsActive() && runTime.seconds() < 1.0 && arm.isBusy()) {
      myOp.idle();
    }
    arm.setPower(0.0);
    return true;
  } // end arm position

  public void closeGate() {
    gate.setPosition(1.0); // / todo set direction close power
  }

  public void openGate() {
    gate.setPosition(0.5); // todo set direction open power
  }

  // auto seek set by range found by the April tag

  // might be combined
  public void flyVelocity(double range) {
    double velocPrs = 0.0;
    if (range > 120) range = 120;
    // fly wheel velocity from range formula
    double velocSeek = (-.007 * range * range) + 4.26 * range + 1396.6;
    spinUp(velocSeek);
    do {
      velocPrs = launch.getVelocity();
      myOp.sleep(20);
    } while (myOp.opModeIsActive()
        && runTime.seconds() < 3.0
        && ((0.95 * velocSeek) > velocPrs ^ (1.05 * velocSeek) < velocPrs));
    // light indicator
  }

  public void setTilt() {
    tilt.setPosition(.5);
  }
}
