// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers.chute;

/**
 * Main entry point for chute control system.
 *
 * <p>Provides high-level interface for controlling a linear actuator (chute) using a motor and
 * potentiometer. Automatically handles homing, position tracking across potentiometer wraparound,
 * and movement to target positions.
 *
 * <h2>Basic Usage (Testing)</h2>
 *
 * <pre>
 * Chute chute = new Chute(3 * Math.PI);  // 4 radians max extension
 *
 * // In your loop
 * chute.update(0.02);  // 50Hz update
 * chute.setTargetPosition(2.0);  // Move to 2 radians
 *
 * if (chute.isAtTarget()) {
 *     // Ready!
 * }
 * </pre>
 *
 * <h2>Usage with Real Hardware</h2>
 *
 * <pre>
 * FtcMotor motor = new FtcMotor(hardwareMap.get(DcMotor.class, "chute_motor"));
 * FtcPotentiometer pot = new FtcPotentiometer(hardwareMap.get(AnalogInput.class, "chute_pot"));
 * ChuteController controller = new ChuteController(motor, pot, 4 * Math.PI);
 * Chute chute = new Chute(controller, motor, pot);
 * </pre>
 *
 * <h2>Features</h2>
 *
 * <ul>
 *   <li>Automatic homing - captures reference voltage at mechanical home position
 *   <li>Position unwrapping - tracks absolute position across pot wraparound
 *   <li>Configurable wraparound - supports different pot ranges (default 2pi)
 *   <li>Auto-homing - automatically homes before first movement if needed
 *   <li>Safety limits - enforces maximum extension
 * </ul>
 *
 * @see ChuteController
 * @see MockPotentiometer
 * @see FtcPotentiometer
 */
public class Chute {
  private final ChuteController controller;
  private final MockMotor motor;
  private final MockPotentiometer pot;

  /**
   * Creates chute with mock hardware for testing. Uses default 2pi (full rotation) potentiometer
   * wraparound.
   *
   * @param maxExtension Maximum extension from home position in radians
   */
  public Chute(double maxExtension) {
    this(maxExtension, 2 * Math.PI);
  }

  /**
   * Creates chute with mock hardware and custom pot wraparound. Use this constructor when testing
   * with non-standard potentiometer ranges.
   *
   * @param maxExtension Maximum extension from home position in radians
   * @param wrapAmount Potentiometer wraparound amount in radians (e.g., pi, 2pi, 4pi)
   */
  public Chute(double maxExtension, double wrapAmount) {
    this.motor = new MockMotor();
    this.pot = new MockPotentiometer(wrapAmount);
    this.controller = new ChuteController(motor, pot, maxExtension);
  }

  /**
   * Creates chute with real FTC hardware adapters. Use this constructor in actual FTC OpModes with
   * hardware from hardwareMap.
   *
   * @param controller Pre-configured controller with real hardware
   * @param motor FtcMotor adapter wrapping DcMotor
   * @param pot FtcPotentiometer adapter wrapping AnalogInput
   * @see FtcMotor
   * @see FtcPotentiometer
   */
  public Chute(ChuteController controller, MockMotor motor, MockPotentiometer pot) {
    this.controller = controller;
    this.motor = motor;
    this.pot = pot;
  }

  /**
   * Updates chute state - must be called every loop iteration. Handles motor movement, pot reading,
   * homing, and position control.
   *
   * @param dt Time since last update in seconds (typically 0.02 for 50Hz)
   */
  public void update(double dt) {
    controller.update(dt);
  }

  /**
   * Starts homing sequence. Drives chute down until pot stops changing, then captures home voltage.
   * Typically not needed as setTargetPosition() auto-homes if required.
   */
  public void home() {
    controller.home();
  }

  /**
   * Commands chute to move to target position. Automatically initiates homing if not already homed.
   *
   * @param position Target position in radians from home (clamped to [0, maxExtension])
   */
  public void setTargetPosition(double position) {
    controller.moveToPosition(position);
  }

  /** Emergency stop - immediately halts motor and cancels any movement. */
  public void stop() {
    controller.stop();
  }

  /**
   * Gets current position of chute.
   *
   * @return Current position in radians from home (0.0 at home position)
   */
  public double getPosition() {
    return controller.getPosition();
  }

  /**
   * Checks if chute has completed homing sequence.
   *
   * @return true if homed and ready for position commands
   */
  public boolean isHomed() {
    return controller.isHomed();
  }

  /**
   * Checks if chute is currently moving to target.
   *
   * @return true if actively moving
   */
  public boolean isMoving() {
    return controller.isMoving();
  }

  /**
   * Checks if chute is homed and stationary at target position.
   *
   * @return true if homed and not moving (ready for next command)
   */
  public boolean isAtTarget() {
    return isHomed() && !isMoving();
  }

  /**
   * Gets motor instance for testing.
   *
   * @return MockMotor instance used by this chute
   */
  public MockMotor getMotor() {
    return motor;
  }

  /**
   * Gets potentiometer instance for testing.
   *
   * @return MockPotentiometer instance used by this chute
   */
  public MockPotentiometer getPot() {
    return pot;
  }
}
