// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode; // Copyright (c) 2024-2025 FTC 13532

// All rights reserved.

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

public class DC_Swerve_Drive {
  private LinearOpMode myOp = null;
  private LynxModule[] allHubs;

  // Define a constructor that allows the OpMode to pass a reference to itself.
  DC_Swerve_Drive(LinearOpMode opmode) {
    myOp = opmode;
  }

  // *** - *** -
  // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
  public DcMotorEx lfDrive = null;
  public DcMotorEx rtDrive = null;
  // ---
  // public Servo lfTurn = null;
  // public Servo rtTurn = null;
  public Servo lfTurn = null;
  public Servo rtTurn = null;

  // ---
  public AnalogInput lfSPot = null;

  public AnalogInput rtSPot = null;

  // ---
  // Swerve chassis constants in inches (to be done)
  private static final double wDia = 96.0; // mm
  private static final double wCir = wDia * 3.141592; // 301.593 wheel circumference
  public static final double mEnc = 537.7; // PPR
  private static final double mRPM = 312; // RPM 5.2 rps
  private static final double cRadius = 203.2; // chassis radius mm from center for turning
  private static final double gearRatio = 1.7; // gear ratio
  private static final double wheelBaseWidth = 298.45;
  private static final double trackWidth = 323.85;
  // outside wheel angle from inside turn aout=ain*ratio
  private final double ackermanRatio = getAckermanRatio(wheelBaseWidth, trackWidth);
  // turn servo constants
  private static final double minTurnRad = Math.toRadians(0.0);
  private static final double maxTurnRad = Math.toRadians(235.0);
  // following 3 determined from UTServeTest
  private static final double servoMin = 0.85;
  private static final double sp180 = 2.52;
  private static final double servoMax = 3.03;
  // turn slope ~= 0.5315 radians provides larger slope + fudge factor
  private static final double turnslope = ((servoMax - servoMin) / (maxTurnRad - minTurnRad)) + 0.0;
  double turnDeg = 3.1415; // radians
  double setServo = turnDeg * turnslope + servoMin;
  public double lfsa, rtsa; // global servo potentiometer voltages
  private double lfVal, rtVal; // start, present position
  public boolean cutSpeed = false;
  // ---
  // KPI PID controller variables
  private static final double Kp = 0.5; // proportional K gain
  private static final double ki = 0.1; // integral K gain
  private final double kd = 0.2; // derivative k gain
  protected boolean anglewrap = false; //
  // pid controllers set set with values above
  DC_PIDController lfController = new DC_PIDController(Kp, ki, kd);
  DC_PIDController rtController = new DC_PIDController(Kp, ki, kd);
  // ---
  private final double stfAng = 0.0; // Saved straf angle
  protected boolean stfDrv = true; // true for drive false for straf

  // ---
  // Global variables -

  // Methods used in Swerve  robot
  // -----------------------------
  public void SwerveInit() {
    // ---
    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();
    // Define and Initialize Motors (note: need to use reference to actual OpMode).
    lfDrive = myOp.hardwareMap.get(DcMotorEx.class, "LFM");
    rtDrive = myOp.hardwareMap.get(DcMotorEx.class, "RFM");

    // To drive forward, most robots need the motor on one side to be reversed, because the axles
    // point in opposite directions.
    // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on
    // your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90
    // Deg drives may require direction flips
    lfDrive.setDirection(DcMotorEx.Direction.FORWARD);
    rtDrive.setDirection(DcMotorEx.Direction.REVERSE);

    // set encoders to zero a stop is performed so set either WITHOUT or WITH encoders
    // if not the motors are stopped and will not start !!!
    lfDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    lfDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    rtDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    rtDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    // use braking to slow the motor down faster
    lfDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    rtDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    // Define and initialize ALL installed servos.
    lfTurn = myOp.hardwareMap.get(Servo.class, "LFS");
    rtTurn = myOp.hardwareMap.get(Servo.class, "RFS");
    // limit the turn range of servos, may not need this
    // lfTurn.scaleRange(0.3, 0.67); // set limits of servo
    // rtTurn.scaleRange(0.3, 0.67); // set limits of servo

    lfSPot = myOp.hardwareMap.get(AnalogInput.class, "LFP");
    rtSPot = myOp.hardwareMap.get(AnalogInput.class, "RFP");

    // setup bulk reads
    List<LynxModule> allHubs = myOp.hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
  }

  // return potentiometer voltage from JoyStick value
  public double gpXtoSpv(double gpX) {
    // map joystick 0 to 1 as 90 to 180 and | -1 to 0 | 180 to 270 return sign to motor drive
    return servoMax + (servoMin - servoMax) * gpX;
  }

  // turn the robot
  // they are treated separate but presently are the same values 90 - about 270
  // passes the seek potentiometer voltages
  public void turnRobot(double lfsp, double rtsp) {
    // check if 225 and 45 turning voltages - turn upon ifself not implemented need is
    // determine shortest path in rotation
    // start deg - end deg > 180 voltage - maxVolt < 180 voltage - maxVolt
    double lfEnd = lfsp - lfVal;
    double rtEnd = rtsp - lfVal;
    // calculate the rotation volts; // potentiometer seek voltage
    if (lfEnd > sp180) lfEnd = lfEnd - servoMax;
    else lfEnd = lfEnd + servoMax;
    if (rtEnd > sp180) rtEnd = rtEnd - servoMax;
    else rtEnd = rtEnd + servoMax;
    // if + rot ccw if - rot cw the abs of result
    // turning 180 - 270 causes the motor to reverse
    // set drive speed at .5
    driveSpeed(.5 * Math.signum(lfEnd));
    driveSpeed(.5 * Math.signum(rtEnd));
    while (myOp.opModeIsActive() && servoBusy(lfEnd, rtEnd)) {
      myOp.idle();
    }
  }

  // set robot to turn around center
  public void posRot() { // position to rotate robot - analog fb avail
    ElapsedTime timer = new ElapsedTime();

    // lfTurn.setPosition(0.43); // offset calibration
    // rtTurn.setPosition(0.57); // wheel at -45 deg

    lfDrive.setDirection(DcMotorEx.Direction.FORWARD);
    rtDrive.setDirection(DcMotorEx.Direction.FORWARD);
    while (myOp.opModeIsActive() && servoBusy(0.43, 0.43)) {
      myOp.idle();
    }
  }

  // check for a change in potentiometer returning false if
  // potentiometer stops when servo reaches its set position
  // use in a while loop call getServoPot before entering while
  public boolean servoBusy(double sclf, double scrt) { // reference state
    // sclf is the seek servo pot voltage
    boolean cmp = true;
    ElapsedTime timer = new ElapsedTime();

    while (myOp.opModeIsActive() && cmp) {
      // state of pot
      boolean lf_f = false, rt_f = false;
      getServoPot(); // read xxsa global variables
      // error lfsa && rtsa are servo potentiometer voltages
      if (Math.abs(sclf - lfsa) < 0.1) lf_f = true;
      if (Math.abs(scrt - rtsa) < 0.1) rt_f = true;
      if (lf_f && rt_f) cmp = false;
    }
    // compare to saved  if a servo has settled otherwise return
    return cmp;
  } // end servoBusy

  public void getServoPot() {
    lfsa = lfSPot.getVoltage();
    rtsa = rtSPot.getVoltage();
  }

  // returns servo cmd set position 0 - 1
  public double potCmd(double pot) {
    double averMax = (lfSPot.getMaxVoltage() + rtSPot.getMaxVoltage()) / 2.0;
    return pot / averMax;
  }

  // returns estimated potentiometer voltage from
  // servo cmd 0 - 1
  private double cmdPot(double cmd) {
    return (2.3373 * cmd) + 0.455;
  }

  // this starts servos moving with CRServo power -1 < 0 < 1
  public void servoTurn(double setServo) {
    lfTurn.setPosition(setServo);
    rtTurn.setPosition(setServo);
  }

  public void driveSpeed(double desV) {
    // Use existing function to drive wheels.
    // left drive
    lfDrive.setPower(desV);
    rtDrive.setPower(desV);
  }

  public void driveStop() {
    lfDrive.setPower(0.0);
    rtDrive.setPower(0.0);
  }

  // calculate distance in turn in encoder pulses as whole pulse; integer
  public int encCntR(int angle) { // encoder count radius turning
    return (int) (gearRatio * mEnc * angle / 360.0);
  }

  // calculate distance to travel in encoder pulses
  public int encCntD(double dist) { // encoder count distance
    return (int) (mEnc * dist / wCir);
  }

  public double getWDist(double tDist) {
    double vecDist = 0.0;
    return wCir * tDist;
  }

  public double getVecLen(double x, double y) {
    return Math.sqrt((x * x) + (y * y));
  }

  public double getVecAng(double x, double y) {
    if (x == 0.0) return 0.0;
    return Math.atan(y / x);
  }

  //  (r,θ), write x=rcosθ and y=rsinθ.
  public double getX(double d, double a) {
    return d * Math.cos(a);
  }

  public double getY(double d, double a) {
    return d * Math.sin(a);
  }

  // Get encoder position value lf motor
  public int getlfEncoder() {
    return lfDrive.getCurrentPosition();
  }

  // Get encoder position value rf motor
  public int getrfEncoder() {
    return rtDrive.getCurrentPosition();
  }

  // Get encoder position value rr motor
  private static double getAckermanRatio(double wheelBaseWidth, double trackWidth) {
    // arbritary radii greater than track width
    double ackRadius = 400; // mm
    double ain = Math.atan(wheelBaseWidth / (ackRadius - trackWidth / 2.0));
    double aout = Math.atan(wheelBaseWidth / (ackRadius + trackWidth / 2.0));
    // speed difference should be added
    return aout / ain;
  }
} // end class Swerve Components
