// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Controls intake, gate, tilt, arm, and launcher hardware. Called from DecodeTeleop via
 * InputStateMachine callbacks.
 */
public class GameDriver {

  // --- Hardware ---
  private final DcMotorEx launch;
  private final DcMotor arm;
  private final CRServo intake;
  private final Servo gate;
  private final Servo tilt;
  private final Telemetry telemetry;

  // --- Gate positions ---
  private static final double GATE_OPEN = 1.0;
  private static final double GATE_CLOSED = 0.5;

  // --- Tilt positions ---
  private static final double TILT_DEFAULT = 0.5;

  // --- Intake state ---
  private boolean intakeRunning = false;

  // --- Arm ---
  private final int armHome;

  public GameDriver(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;

    launch = hardwareMap.get(DcMotorEx.class, "launch");
    launch.setDirection(DcMotorSimple.Direction.FORWARD);
    launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    arm = hardwareMap.get(DcMotor.class, "arm");
    arm.setDirection(DcMotorSimple.Direction.FORWARD);
    armHome = arm.getCurrentPosition();

    intake = hardwareMap.get(CRServo.class, "intake");
    gate = hardwareMap.get(Servo.class, "gate");
    tilt = hardwareMap.get(Servo.class, "tilt");
  }

  // -----------------------------------------------------------------------
  // Intake
  // -----------------------------------------------------------------------

  /** Toggle intake on/off. */
  public void toggleIntake() {
    intakeRunning = !intakeRunning;
    intake.setPower(intakeRunning ? -1.0 : 0.0);
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
    gate.setPosition(GATE_OPEN);
    launch.setPower(power);
  }

  /** Spin flywheel to a target velocity. */
  public void setLaunchVelocity(double velocity) {
    launch.setVelocity(velocity);
  }

  /** Stop flywheel. */
  public void launchOff() {
    launch.setVelocity(0.0);
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
}
