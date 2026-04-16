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
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

  public DcMotor intake = null; // intake motor controller

  public DcMotor hoodEncoder = null;
  public CRServo chuteMotor = null;
  public Servo gate = null; // intake gate
  public Servo tilt = null; // tilt robot up
  public DigitalChannel home = null;

  // hood perimeters calculated from segment chord and sagi to rev counts
    private static final int encCtsPerInch = 922;
    private static final int enCctsMax = 9759;

  // time out timer
  private ElapsedTime runTime = new ElapsedTime();
  // global variables
  public int encHome = 0;

  int homeenc = 0; // start voltage
  // does hoodPos = home position
  int hoodPos = 0;
  // hood parameters
  int minChutePos = homeenc;
  boolean homePosSet = false;

  // status light

  public void InitIL() {

    // Define and Initialize Motor.
    launch = myOp.hardwareMap.get(DcMotorEx.class, "launch"); // Flywheel
    launch.setDirection(DcMotorSimple.Direction.FORWARD);
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // prepare use velocity
    arm =
        myOp.hardwareMap.get(
            DcMotor.class,
            "arm"); // Bar attached to flywheel, is set to correct tilt at start of match
    // Arm needs to lower at end of match
    arm.setDirection(DcMotorSimple.Direction.FORWARD);

    // Define and Initialize Servo
    intake =
        myOp.hardwareMap.get(
            DcMotor.class, "Intake"); // Rubber bar and rubber gears to pull ball toward gate
    intake.setDirection(DcMotorSimple.Direction.REVERSE);
    // gate to stop balls from entering the chute
    gate =
        myOp.hardwareMap.get(
            Servo.class, "gate"); // Servo located in front of shooter to control hopper
    tilt =
        myOp.hardwareMap.get(
            Servo.class, "tilt"); // Bar attached to camera (not used for this software feature)
    // present.SensorInit();
    encHome = arm.getCurrentPosition(); // Not used for this software feature
    // initialize hood components
    chuteMotor = myOp.hardwareMap.get(CRServo.class, "chute"); // Not used for this software
    hoodEncoder = myOp.hardwareMap.get(DcMotor.class, "hoodencoder");

    home = myOp.hardwareMap.get(DigitalChannel.class, "Home"); // Not used for this software

    ElapsedTime timeOut = new ElapsedTime();
    if (!home.getState()) {
      timeOut.reset();
      chuteMotor.setPower(.1); // set proper direction
      while ((timeOut.seconds() < 5.0) && home.getState()) {
        myOp.idle();
      }
      homePosSet = true;
      if (timeOut.seconds() > 5) {
        myOp.telemetry.addLine("Hood home error");
        myOp.telemetry.update();
        homePosSet = false;
      }
    } // end if home
    homeenc = hoodEncoder.getCurrentPosition();
  } // end of InitL

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
    return ready;
  } // end spin up

  public void spinOff() {
    launch.setVelocity(0.0);
  }

  /*
   launch arm has two positions
   launch  position to launch ball
   endGame end game retraction

   Autonomous must start at launch position
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

  public int getArmEnc() {
    return arm.getCurrentPosition();
  }

  public void closeGate() {
    gate.setPosition(0.5); // / todo set direction close power
  }

  public void openGate() {
    gate.setPosition(1.0); // todo set direction open power
  }

  // auto seek set by range found by the April tag
  public double getFlyVel() {
    return launch.getVelocity();
  }

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

  // must use +/- encoder count to move
  public void hoodSetPos(int gotoHoodPos) {
    int gotoPos = gotoHoodPos;
    if (gotoPos < homeenc) gotoPos = homeenc;
    if (gotoPos > enCctsMax) gotoPos = enCctsMax;
    int encPos = hoodEncoder.getCurrentPosition();
    hoodEncoder.setTargetPosition(gotoPos);
    hoodEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    chuteMotor.setPower(.6);
      while (myOp.opModeIsActive() && hoodEncoder.isBusy()) {
            myOp.idle();
      }

  }

  public boolean getHome() {
    return home.getState();
  }

  public int getEncCtPerInch(){
      return encCtsPerInch;
    }

} // DcIntake_Launch
