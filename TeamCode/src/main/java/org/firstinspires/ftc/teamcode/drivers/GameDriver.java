// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivers.wpilib.interpolation.InterpolatingDoubleTreeMap;

/**
 * Controls intake, gate, tilt, arm, and launcher hardware. Called from DecodeTeleop via
 * InputStateMachine callbacks.
 */
public class GameDriver {

  // --- Hardware ---
  private final DcMotorEx launch;
  private final DcMotor arm;
  private final DcMotor intake;
  private final DcMotor hoodEncoder;
  private final Servo gate;
  private final Servo tilt;
  private final Telemetry telemetry;
  private final LinearOpMode linOp;

  private final DigitalChannel touchSensor;

  // --- Gate positions ---
  private static final double GATE_OPEN = 1.0;
  private static final double GATE_CLOSED = 0.5;

  // --- Tilt positions ---
  private static final double TILT_DEFAULT = 0.5;

  // --- Intake state ---
  private boolean intakeRunning = false;

  // --- Arm ---
  private final int armHome;
  // servo power and pid error
  private static double servode = 0.0;
  private static final int revCtperRev = 8192;
  private static final double gearDia = 1.415;
  private static final double gearCircum = gearDia * 3.141;

  public GameDriver(HardwareMap hardwareMap, Telemetry telemetry) {
    this.linOp = null;

    this.telemetry = telemetry;
    launch = hardwareMap.get(DcMotorEx.class, "launch");
    launch.setDirection(DcMotorSimple.Direction.FORWARD);
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    launch.setZeroPowerBehavior(
        DcMotor.ZeroPowerBehavior.BRAKE); // break hard could pull battery down
    arm = hardwareMap.get(DcMotor.class, "arm");
    arm.setDirection(DcMotorSimple.Direction.FORWARD);
    armHome = arm.getCurrentPosition();
    hood = hardwareMap.get(CRServo.class, "hood");
    hood.setDirection(DcMotorSimple.Direction.FORWARD);
    intake = hardwareMap.get(DcMotor.class, "intake");
    intake.setDirection(DcMotorSimple.Direction.REVERSE);
    gate = hardwareMap.get(Servo.class, "gate");
    tilt = hardwareMap.get(Servo.class, "tilt");

    hoodEncoder = hardwareMap.get(DcMotor.class, "Hood");
    hoodEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    touchSensor = hardwareMap.get(DigitalChannel.class, "Home");
  }

  public GameDriver(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linOp) {

    this.linOp = linOp;

    this.telemetry = telemetry;
    launch = hardwareMap.get(DcMotorEx.class, "launch");
    launch.setDirection(DcMotorSimple.Direction.FORWARD);
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    launch.setZeroPowerBehavior(
        DcMotor.ZeroPowerBehavior.BRAKE); // break hard could pull battery down
    arm = hardwareMap.get(DcMotor.class, "arm");
    arm.setDirection(DcMotorSimple.Direction.FORWARD);
    armHome = arm.getCurrentPosition();
    hood = hardwareMap.get(CRServo.class, "hood");
    hood.setDirection(DcMotorSimple.Direction.FORWARD);
    intake = hardwareMap.get(DcMotor.class, "intake");
    intake.setDirection(DcMotorSimple.Direction.REVERSE);
    gate = hardwareMap.get(Servo.class, "gate");
    tilt = hardwareMap.get(Servo.class, "tilt");

    hoodEncoder = hardwareMap.get(DcMotor.class, "Hood");
    hoodEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    touchSensor = hardwareMap.get(DigitalChannel.class, "Home");
  }

  private CRServo hood = null;
  public TouchSensor bob = null;
  int homeEncoder = 0;

  int homedirection = 1; // neg 1 for - power to home
  int currentPosition = 0;
  int targetPosition = 0;
  private final ElapsedTime eTime = new ElapsedTime();
  private final ElapsedTime dTime = new ElapsedTime();

  // -----------------------------------------------------------------------
  // Intake
  // -----------------------------------------------------------------------

  /** Toggle intake on/off. */
  public void toggleIntake() {
    intakeRunning = !intakeRunning;
    intake.setPower(intakeRunning ? -1.0 : 0.0);
  }

  public void setIntakePower(float power) {
    if (power < 0.01) {
      intakeRunning = false;
      power = 0.0f;
    } else {
      intakeRunning = true;
    }

    // Power needs to be inverted before applying to intake motor
    intake.setPower(-1.0 * power);
  }

  /** Run intake forward. */
  public void intakeOn() {
    intakeRunning = true;
    intake.setPower(-1.0);
  }

  /** Stop intake. */
  public void intakeOff() {
    intakeRunning = false;
    intake.setPower(0.0);
  }

  public boolean isIntakeRunning() {
    return intakeRunning;
  }

  // -----------------------------------------------------------------------
  // Gate
  // -----------------------------------------------------------------------

  public void openGate() {
    gate.setPosition(GATE_OPEN);
  }

  public void closeGate() {
    gate.setPosition(GATE_CLOSED);
  }

  // -----------------------------------------------------------------------
  // Tilt
  // -----------------------------------------------------------------------

  public void setTilt(double position) {
    tilt.setPosition(position);
  }

  public void tiltDefault() {
    tilt.setPosition(TILT_DEFAULT);
  }

  // -----------------------------------------------------------------------
  // Arm -- direct power control from joystick
  // -----------------------------------------------------------------------

  /** Set arm power directly (pass joystick Y value). */
  public void setArmPower(double power) {
    arm.setPower(power);
  }

  public double getArmPosition() {
    return arm.getCurrentPosition();
  }

  // -----------------------------------------------------------------------
  // Launcher
  // -----------------------------------------------------------------------
  // feedback flywheel velocity
  public double getFlyVelocity() {
    return launch.getVelocity();
  }

  /** Set flywheel power directly (pass joystick value). */
  public void setLaunchPower(double power) {
    // gate.setPosition(GATE_OPEN);
    launch.setPower(power);
  }

  public void setIntakePower(double power) {
    intake.setPower(-power);
  }

  /** Spin flywheel to a target velocity. */
  public void setLaunchVelocity(double velocity) {
    launch.setVelocity(velocity);
  }

  /** Stop flywheel. */
  public void launchOff() {
    launch.setVelocity(0.0);
  }

  public void HoodPosition(int newPosition) {
    try {
      // target position!
      dTime.reset(); // differential reset
      eTime.reset(); // udp reset
      while (linOp.opModeIsActive() && IsBusy(newPosition)) {
        linOp.idle();
      }
      hood.setPower(0.0);

    } catch (Exception e) {
      e.printStackTrace();
    }
  } // end hood position

  // range is the camera range in inches from target or gamepad -1 to 1 converted to 20 to 130
  // inches
  public int EncCntfrmRange(double range) {
    InterpolatingDoubleTreeMap distanceToHoodMap = new InterpolatingDoubleTreeMap();
    distanceToHoodMap.put(43.0, 2.0);
    distanceToHoodMap.put(79.0, 4.0);
    distanceToHoodMap.put(126.0, 8.0);
    distanceToHoodMap.put(130.0, 8.5);
    distanceToHoodMap.put(160.0, 9.5);
    distanceToHoodMap.put(296.0, 10.5);
    final double Hm = 10.5; // maximum hood height
    final int Hmc = (int) (revCtperRev * Hm / gearCircum); // max hood encoder count
    double Hh = distanceToHoodMap.get(range); // desired hood height from range
    int hood = (int) (revCtperRev * Hh / gearCircum); // hood desired count position
    return hood;
  }

  private boolean IsBusy(int target) {
    final double k = 0.01;
    final double d = .005;
    int encCt = 0; // encoder goes here
    int sgn = target > encCt ? -1 : 1; // change signs for reverse movement
    double tdiff = encCt - target;
    double servoPwr = sgn * tdiff * k + d * (tdiff - servode) / dTime.milliseconds();
    hood.setPower(Range.clip(servoPwr, -1, 1));
    servode = tdiff; // save error for differential error
    dTime.reset(); // reset for differential
    return !(tdiff < 1);
  }

  // Move hood to home if not already there and reset encoder to 0
  public void GotoHome() {
    double hmpwr = .2;
    if (hmpwr > 0) homedirection = 1;
    else homedirection = -1;
    hood.setPower(hmpwr);
    while (linOp.opModeIsActive() && !bob.isPressed()) {
      linOp.idle();
    }
    hood.setPower(0.0);
    homeEncoder = 0; // encoder needs to be added
  }

  public double gp2LYjoy() {
    return linOp.gamepad2.left_stick_y;
  }

  public double gp2LXjoy() {
    return linOp.gamepad2.left_stick_x;
  }

  public double gp2YRjoy() {
    return linOp.gamepad2.right_stick_y;
  }

  public double gp2Xjoy() {
    return linOp.gamepad2.left_stick_x;
  }

  // -----------------------------------------------------------------------
  // Telemetry
  // -----------------------------------------------------------------------

  public void updateTelemetry() {
    telemetry.addData("Intake", intakeRunning ? "ON" : "OFF");
    telemetry.addData("Gate", gate.getPosition() >= GATE_OPEN ? "OPEN" : "CLOSED");
    telemetry.addData("Arm Power", "%.2f", arm.getPower());
    telemetry.addData("Launch Vel", "%.0f", launch.getVelocity());
  }

  public double getHoodEncoderPos() {
    return hoodEncoder.getCurrentPosition();
  }

  public boolean getTouchSensor() {
    return touchSensor.getState();
  }

  public void setHoodServoPower(double power) {
    hood.setPower(power);
  }

  public void resetHoodEncoder() {
    hoodEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  public void setGate(double pos) {
    gate.setPosition(pos);
  }
}
