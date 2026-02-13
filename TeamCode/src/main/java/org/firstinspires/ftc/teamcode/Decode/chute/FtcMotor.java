// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode.chute;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Adapter that wraps a real FTC DcMotor and makes it look like a MockMotor. Use this to connect the
 * chute controller to actual hardware.
 */
public class FtcMotor extends MockMotor {
  private final DcMotor motor;

  /**
   * Create motor adapter.
   *
   * @param motor The real FTC motor from hardwareMap
   */
  public FtcMotor(DcMotor motor) {
    super();
    this.motor = motor;

    // Configure motor
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  @Override
  public void setPower(double power) {
    super.setPower(power); // Track power in parent
    motor.setPower(power); // Send to real motor
  }

  @Override
  public void update(double dt) {
    // Real motor updates itself - we don't simulate
    // But we still call parent to track internal position for mock purposes
    super.update(dt);
  }
}
