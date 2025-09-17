// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mekanism {

  LinearOpMode myOp;

  public final DcMotorEx pivot;
  public final DcMotorEx slide, slide2;

  private final DigitalChannel limitSwitch;

  private final Servo intakeServo, intakeServo2;
  private final Servo wrist;

  private final Servo ramp1, ramp2;

  public final int limitSlide = 4200;
  public final double limitPivot = 3500;
  public final double countsPerDegree = 41.855;
  public final double countsPerInch = 120.88;

  public double slideTarget = 0;

  private boolean wristMoved = false;

  private final Telemetry telemetry;

  ElapsedTime pivotTimer = new ElapsedTime();


  public Mekanism(LinearOpMode opMode) {
    // Init slaw, claw, and pivot
    pivot = (DcMotorEx) opMode.hardwareMap.dcMotor.get("pivot");
    slide = (DcMotorEx) opMode.hardwareMap.dcMotor.get("slide");
    slide2 = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "slide 2");

    pivot.setMode(STOP_AND_RESET_ENCODER);
    slide.setMode(STOP_AND_RESET_ENCODER);
    slide2.setMode(STOP_AND_RESET_ENCODER);

    pivot.setTargetPosition(0);
    slide.setTargetPosition(0);
    slide2.setTargetPosition(0);

    pivot.setMode(RUN_TO_POSITION);
    slide.setMode(RUN_TO_POSITION);
    slide2.setMode(RUN_TO_POSITION);

    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    pivot.setDirection(DcMotorSimple.Direction.FORWARD);
    slide.setDirection(DcMotorSimple.Direction.FORWARD);
    slide2.setDirection(DcMotorSimple.Direction.FORWARD);

    pivot.setPower(1);
    slide.setPower(1);
    slide2.setPower(1);


    limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "limit switch");
    limitSwitch.setMode(DigitalChannel.Mode.INPUT);

    // Servos for intake
    intakeServo = opMode.hardwareMap.get(Servo.class, "intake");
    intakeServo2 = opMode.hardwareMap.get(Servo.class, "intake2");
    wrist = opMode.hardwareMap.get(Servo.class, "wrist");

    intakeServo.setDirection(REVERSE);
    intakeServo2.setDirection(Servo.Direction.FORWARD);
    wrist.setDirection(REVERSE);

    intakeServo.setPosition(0.5);
    intakeServo2.setPosition(0.5);
    wrist.setPosition(0.65);


    // Servos for clipper
    ramp1 = opMode.hardwareMap.get(Servo.class, "ramp 1");
    ramp2 = opMode.hardwareMap.get(Servo.class, "ramp 2");

    ramp1.setDirection(Servo.Direction.FORWARD);
    ramp2.setDirection(Servo.Direction.FORWARD);

    unclamp();

    ramp1.scaleRange(0, 1);
    ramp2.scaleRange(0, 1);

    telemetry = opMode.telemetry;

    myOp = opMode;
  }


  // To extend arm, input from game pad 2 straight in
  public void setSlide(int x) {

//    telemetry.addData("slide current pos", slide.getCurrentPosition());

    //angle measure thing
    double maxLength = limitSlide * (Math.cos(Math.toRadians(pivot.getCurrentPosition() / countsPerDegree)) * 1.05);
    if(maxLength<2500)
      maxLength = 2500;
    if(maxLength>limitSlide)
      maxLength = limitSlide;
    if(x>maxLength)
      x = (int) maxLength;
    if(x<0)
      x = 0;
    slideTarget = x;

    slide.setTargetPosition(x);
    slide2.setTargetPosition(x);

    if(!slide.isBusy())
      slide2.setPower(0);
  }


  public void setPivot(double x, boolean raiseLimit) {
    double current_pos = pivot.getCurrentPosition();
    if(raiseLimit)
      current_pos+=500;
    if (current_pos >= limitPivot && x > 0 ) {
      x = 0;
      telemetry.addLine("Current pos over limit");
    } else if (pivot.getCurrentPosition() <= 0 && x < 0) {
      x = 0;
      telemetry.addLine("Current pos under 0");
    }
//    telemetry.addData("Pivot current pos", pivot.getCurrentPosition());
//    telemetry.addData("Limit switch: ",limitSwitch.getState());

    x *= .5;
    pivot.setPower(x);
    if(x > 0)
      pivot.setTargetPosition(4000);
    else
      pivot.setTargetPosition(0);
  }


  public void homeArm() {
    pivotTimer.reset();

    pivot.setMode(RUN_USING_ENCODER);
    slide.setMode(RUN_USING_ENCODER);
    slide2.setMode(RUN_USING_ENCODER);

    while (limitSwitch.getState() && myOp.opModeIsActive()) {
      telemetry.addData("limit switch: ",limitSwitch.getState());
      pivot.setPower(-.5);
      slide.setPower(-0.5);
      slide2.setPower(-0.5);

      telemetry.addData("Pivot Pos: ", pivot.getCurrentPosition());
      telemetry.update();
    }
    pivot.setPower(0.1);
    myOp.sleep(100);
    pivot.setPower(0.0);
    slide.setPower(0.0);
    slide2.setPower(0.0);
    try {
      Thread.sleep(250);
    } catch (final InterruptedException ex) {
      Thread.currentThread().interrupt();
    }

    pivot.setMode(STOP_AND_RESET_ENCODER);
    slide.setMode(STOP_AND_RESET_ENCODER);
    slide2.setMode(STOP_AND_RESET_ENCODER);

    pivot.setMode(RUN_USING_ENCODER);
    slide.setMode(RUN_TO_POSITION);
    slide2.setMode(RUN_TO_POSITION);

    pivot.setPower(0.0);
    slide.setPower(1.0);
    slide2.setPower(1.0);
  }

  /**
   * 1,0: Intake
   * 0,1: Outtake
   * 1,1: Outtake
   * 0,0: Stop
   */
  public void runIntake(boolean intake, boolean outtake) {
    if (outtake) {
      intakeServo.setPosition(1);
      intakeServo2.setPosition(.625);
    } else if (intake) {
      intakeServo.setPosition(0);
      intakeServo2.setPosition(0);
    } else {
      intakeServo.setPosition(0.5);
      intakeServo2.setPosition(0.5);
    }
  }


  public void clamp() {
    ramp1.setPosition(0);
    ramp2.setPosition(.15);
  }


  public void unclamp() {
    ramp1.setPosition(.15);
    ramp2.setPosition(0);
  }


  public void autoClip() {
    telemetry.addData("pivot current pos", pivot.getCurrentPosition());
    telemetry.addData("slide current pos", slide.getCurrentPosition());
    if (slide.getCurrentPosition() > 150) {
      unclamp();
      slide.setPower(-1);
      wrist.setPosition(0.1);
    } else {
      clamp();
      slide.setPower(0);
      if (pivot.getCurrentPosition() < 2200) {
        wrist.setPosition(0.1);
        pivot.setPower(1);
      } else {
        wrist.setPosition(0);
        slide.setPower(-1);
        if (pivot.getCurrentPosition() < 2500) {
          pivot.setPower(1);
        } else {
          pivot.setPower(0);
        }
      }
    }
    wristMoved = true;
  }


  public void toggleWrist() {
    if (!wristMoved) {
      wrist.setPosition(0.1);
      wristMoved = true;
    } else {
      wrist.setPosition(0.65);
      wristMoved = false;
    }
  }

  public void topSpecimenRung() {
    // setPivot();
    // setSlide();
  }
}
