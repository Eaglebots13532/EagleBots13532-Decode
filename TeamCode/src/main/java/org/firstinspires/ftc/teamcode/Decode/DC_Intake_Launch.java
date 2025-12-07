// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

public class DC_Intake_Launch {
  /* Declare OpMode members.
   * gain access to methods in the calling OpMode.
   */
  private LinearOpMode myOp = null;

  // Default constructor
  public DC_Intake_Launch(LinearOpMode opmode) {
    myOp = opmode;
    present = new DC_BallSensor(myOp);
  }

  public DC_BallSensor present;

  public DcMotorEx launch = null; // 6000 rpm motor
  public DcMotor arm = null; // 312 rpm motor
  public CRServo chute = null;
  public CRServo intake = null; // intake motor controller
  public Servo gate = null; // intake gate
  public AnalogInput chuteVal = null; // chute servo potentiometer
  public Servo tilt = null; // tilt robot up
  // time out timer
  private ElapsedTime runTime = new ElapsedTime();
  // global variables
  public int encHome = 0;

  // status light
  private final RevBlinkinLedDriver status = null;
  private final RevBlinkinLedDriver.BlinkinPattern pattern = null;
  private Deadline ledCycleDeadline;

  DisplayKind displayKind;

  protected enum DisplayKind {
    MANUAL,
    AUTO
  }

  public void InitIL() {

    // Define and Initialize Motor.
    launch = myOp.hardwareMap.get(DcMotorEx.class, "launch");
    launch.setDirection(DcMotorSimple.Direction.FORWARD); // todo set launch direction
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // prepare use velocity
    arm = myOp.hardwareMap.get(DcMotor.class, "arm");
    arm.setDirection(DcMotorSimple.Direction.FORWARD); // todo set arm direction
    chute = myOp.hardwareMap.get(CRServo.class, "chute"); //  todo set chute home
    chuteVal = myOp.hardwareMap.get(AnalogInput.class, "CP");
    // Define and Initialize Servo
    intake = myOp.hardwareMap.get(CRServo.class, "intake");
    // gate to stop balls from entering the chute
    gate = myOp.hardwareMap.get(Servo.class, "gate"); // todo set gate position
    tilt = myOp.hardwareMap.get(Servo.class, "tilt");
    present.SensorInit();
    encHome = arm.getCurrentPosition(); // arm starts in home position

    // status = myOp.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    // pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    // status.setPattern(pattern);
  }

  // Servo controlled motor
  public void Intake() {
    intake.setPower(1.0); // front intake servo motor
  }

  public void IntakeStop() {
    intake.setPower(0.0); // front intake servo motor
  }

  public void launchVelocity() {
    launch.setVelocity(1000.0);
    myOp.sleep(2000);
    myOp.telemetry.addData("Current Velocity", launch.getVelocity());
    myOp.telemetry.update();
    launch.setVelocity(0.0);
  }

  // end motor checks

  public boolean spinUp(double shoot_velocity) {
    boolean ready = true; // the not of false
    launch.setVelocity(shoot_velocity);
    runTime.reset();
    while (myOp.opModeIsActive() && runTime.seconds() < 1.0 && ready) {
      myOp.telemetry.addData("Current Velocity", launch.getVelocity());
      myOp.telemetry.update();
      if (launch.getVelocity() > shoot_velocity) ready = false;
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
    final int launch = encHome + 20; // set + value to encoder count
    final int endGame = encHome + 40;

    int move = 0;
    if (pos == 0) move = home;
    else if (pos == 1) move = launch;
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
    gate.setPosition(0.5); // / todo set direction close power
  }

  public void openGate() {
    gate.setPosition(0.3); // todo set direction open power
  }

  public void chuteHm() {
    seekPos(3.2);
  }

  public void chute60() {
    seekPos(0.5);
  }

  // chute is controlled by driver (teleOp)
  public void chutectl(double pos) {
    seekPos(pos);
  }

  // move chute angle
  // pos is -1 to 1 where seek volt = ((pos + 1.0)/2.0) * potMax
  private void seekPos(double pos) {
    double potMax = 3.3; // set to maximum
    double normVolt = chuteVal.getVoltage() / potMax;
    double seekVolt = (pos + 1.0) / 2.0; // 0 - 1
    double servoPwr = .7;
    // (pos + 1.0)/2.0) * potMax
    runTime.reset();
    servoPwr = (pos > normVolt) ? servoPwr : -servoPwr;
    chute.setPower(servoPwr);
    while (myOp.opModeIsActive() && runTime.seconds() < 2.0 && seekVolt < normVolt) {
      normVolt = chuteVal.getVoltage() / potMax;
      chute.setPower(pos);
    }
    chute.setPower(0.0);
  } // end seek position

  public void setTilt() {
    tilt.setPosition(.5);
  }
}
