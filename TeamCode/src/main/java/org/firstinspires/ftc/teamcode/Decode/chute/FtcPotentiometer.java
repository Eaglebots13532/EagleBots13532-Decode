// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode.chute;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Adapter that wraps a real FTC AnalogInput (potentiometer) and makes it look like a
 * MockPotentiometer. Use this to connect the chute controller to actual hardware.
 */
public class FtcPotentiometer extends MockPotentiometer {
  private final AnalogInput crspot;
  private final double maxVoltage;

  /**
   * Create potentiometer adapter with default 2pi wraparound.
   *
   * @param crspot The real FTC analog input from hardwareMap
   */
  public FtcPotentiometer(AnalogInput crspot) {
    this(crspot, 2 * Math.PI);
  }

  /**
   * Create potentiometer adapter with custom wraparound.
   *
   * @param pot The real FTC analog input from hardwareMap
   * @param wrapAmount Voltage value at which pot wraps (in radians)
   */
  public FtcPotentiometer(AnalogInput pot, double wrapAmount) {
    super(wrapAmount);
    this.crspot = pot;
    this.maxVoltage = pot.getMaxVoltage(); // Usually 6.18V
  }

  @Override
  public double getVoltage() {
    // Read actual voltage from hardware
    double rawVoltage = crspot.getVoltage();

    // Scale to [0, wrapAmount] range
    // Assumes pot uses full voltage range (0 to maxVoltage)
    return (rawVoltage / maxVoltage) * getWrapAmount();
  }

  @Override
  public void updatePosition(double delta) {
    // Real pot updates itself - ignore this
    // (This method is only used in simulation)
  }

  @Override
  public void setPosition(double pos) {
    // Can't set real hardware position - ignore
    // (This method is only used in simulation)
  }
}
