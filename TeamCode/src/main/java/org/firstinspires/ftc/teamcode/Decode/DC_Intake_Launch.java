// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.drivers.chute.Chute;
import org.firstinspires.ftc.teamcode.drivers.chute.ChuteController;
import org.firstinspires.ftc.teamcode.drivers.chute.FtcCRServo;
import org.firstinspires.ftc.teamcode.drivers.chute.FtcPotentiometer;

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
  public Chute chute;

  public CRServo intake = null; // intake motor controller
  public Servo gate = null; // intake gate
  public Servo tilt = null; // tilt robot up
  public CRServo chuteMotor = null;
  public AnalogInput chutePot = null;

  // time out timer
  private ElapsedTime runTime = new ElapsedTime();
  // global variables
  public int encHome = 0;
  boolean potage = false;
  double maxPot = 2.0 * Math.PI; // Was 6.16
  double lastVolt = 0.0;
  double home = 0.0; // start voltage
  // does hoodPos = home position
  double hoodPos = 0.0;
  int count = 0;
  // hood parameters
  private static final double POT_WRAP_AMOUNT = 6.16;
  private static final double MAX = 4 * Math.PI;
  double minChutePos = 0.0;
  boolean homePosSet = false;
  boolean inUpperRegion = false;
  double prevVoltPot = 0.0;
  int majorLoopCt = 0;
  double absChutePos = 0.0;

  // potentiometer voltage change
  double hoodpwr = 0.0;
  double step = 0.0;
  double voltpot = 0.0;
  double correctedChutePos = 0.0;

  // status light

  public void InitIL() {

    // Define and Initialize Motor.
    launch = myOp.hardwareMap.get(DcMotorEx.class, "launch"); // Flywheel
    launch.setDirection(DcMotorSimple.Direction.FORWARD);
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // prepare use velocity
    arm =
        myOp.hardwareMap.get(
            DcMotor.class,
            "arm"); // Bar attached to flywheel, defaults to correct tilt at start of match, needs
    // to lower at end of match
    arm.setDirection(DcMotorSimple.Direction.FORWARD);

    // Define and Initialize Servo
    intake =
        myOp.hardwareMap.get(
            CRServo.class, "intake"); // Rubberband bar and rubber gears to pull ball toward gate
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
    chuteMotor = myOp.hardwareMap.get(CRServo.class, "chute"); // Not used for this software feature
    chutePot = myOp.hardwareMap.get(AnalogInput.class, "CP"); // Not used for this software feature

    // Create hardware adapters
    FtcCRServo motor = new FtcCRServo(chuteMotor);
    FtcPotentiometer pot = new FtcPotentiometer(chutePot, POT_WRAP_AMOUNT);

    // Create chute controller with real hardware
    ChuteController controller = new ChuteController(motor, pot, MAX);
    chute = new Chute(controller, motor, pot);
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
    gate.setPosition(0.5); // / todo set direction close power
  }

  public void openGate() {
    gate.setPosition(1.0); // todo set direction open power
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

  void runToHomePos(CRServo chuteMotor, double home, FtcPotentiometer pot) {
    double newVoltpot = Math.abs(pot.getVoltage());
    int stallCount = 0;
    double prevChutePos = 0.0;

    do {
      if (correctedChutePos < 2.0) {
        chuteMotor.setPower(-0.3);
      } else {
        chuteMotor.setPower(-0.8);
      }
      updatePos(false, home, pot);

      // See if we've stalled, if so, increase a count and make sure
      if ((int) correctedChutePos == (int) prevChutePos) {
        stallCount++;
      } else {
        stallCount = 0;
      }

      prevChutePos = correctedChutePos;

      myOp.sleep(10);
    } while (stallCount <= 50); // Loop until we stall at home

    myOp.telemetry.addLine("Found home");
    myOp.telemetry.update();
    // Reset minChutePos to the new home position
    minChutePos = absChutePos;

    chuteMotor.setPower(0.0);
  }

  Optional<Double> chutePos = Optional.empty();

  private void updatePos(boolean directionUp, double home, FtcPotentiometer pot) {
    double newVoltpot = Math.abs(pot.getVoltage());

    if (directionUp) {
      if (newVoltpot >= maxPot / 2.0) {
        if (!inUpperRegion) {
          inUpperRegion = true;
          majorLoopCt += 1;
        }
      }
      if (newVoltpot < maxPot / 2.0) {
        if (inUpperRegion) {
          inUpperRegion = false;
        }
      }
    } else {
      if (newVoltpot >= maxPot / 2.0) {
        if (!inUpperRegion) {
          inUpperRegion = true;
        }
      }
      if (newVoltpot < maxPot / 2.0) {
        if (inUpperRegion) {
          inUpperRegion = false;
          majorLoopCt -= 1;
        }
      }
    }

    absChutePos = (maxPot * majorLoopCt) + (maxPot - newVoltpot);
    if (!homePosSet) {
      homePosSet = true;
      minChutePos = absChutePos;
    }
    correctedChutePos = absChutePos - minChutePos;
    /*
           telemetry.addLine("Potentiometer: " + newVoltpot);
           telemetry.addLine("Loop ct: " + majorLoopCt);
           telemetry.addLine("-- Transition: " + maxPot / 2.0);
           if (inUpperRegion) {
               telemetry.addLine("Region: UPPER");
           } else {
               telemetry.addLine("Region: LOWER");
           }
           telemetry.addLine("Chute POS: " + absChutePos);
           telemetry.addLine("Corrected POS: " + correctedChutePos);
           telemetry.addData("Potentiometer", pot.getVoltage());
           telemetry.update();

    */

    prevVoltPot = newVoltpot;
  }

  public void setTilt() {
    tilt.setPosition(.5);
  }
}
