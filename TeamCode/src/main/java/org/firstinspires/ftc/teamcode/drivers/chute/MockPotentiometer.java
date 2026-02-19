// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers.chute;

/** Simple mock potentiometer with configurable wraparound. */
public class MockPotentiometer {
  private double position;
  private final double wrapAmount;

  /** Create potentiometer with default 2pi wraparound. */
  public MockPotentiometer() {
    this(2 * Math.PI);
  }

  /**
   * Create potentiometer with custom wraparound amount.
   *
   * @param wrapAmount Voltage at which pot wraps back to 0
   */
  public MockPotentiometer(double wrapAmount) {
    this.position = 0.0;
    this.wrapAmount = wrapAmount;
  }

  public double getVoltage() {
    // Wrap to [0, wrapAmount]
    double wrapped = position % wrapAmount;
    if (wrapped < 0) wrapped += wrapAmount;
    return wrapped;
  }

  public void setPosition(double pos) {
    this.position = pos;
  }

  public void updatePosition(double delta) {
    this.position += delta;
  }

  public double getWrapAmount() {
    return wrapAmount;
  }
}
