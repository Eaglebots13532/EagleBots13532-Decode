// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.chute;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Adapter that wraps a real FTC DcMotor and makes it look like a MockMotor. Use this to connect the
 * chute controller to actual hardware.
 */
public class FtcCRServo extends MockMotor {
  private final CRServo crservo;

  /**
   * Create crservo adapter.
   *
   * @param crservo The real FTC crservo from hardwareMap
   */
  public FtcCRServo(CRServo crservo) {
    super();
    this.crservo = crservo;
  }

  @Override
  public void setPower(double power) {
    super.setPower(power); // Track power in parent
    crservo.setPower(power); // Send to real motor
  }

  @Override
  public void update(double dt) {
    // Real motor updates itself - we don't simulate
    // But we still call parent to track internal position for mock purposes
    super.update(dt);
  }
}
