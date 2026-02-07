// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.subsystems.chute;

/** Simple mock motor. */
public class MockMotor {
  private double power;
  private double position;
  private double speed = 1.0; // rad/s at full power

  public MockMotor() {
    this.power = 0.0;
    this.position = 0.0;
  }

  public void setPower(double power) {
    this.power = Math.max(-1.0, Math.min(1.0, power));
  }

  public double getPower() {
    return power;
  }

  public void update(double dt) {
    position += power * speed * dt;
    if (position < 0) position = 0; // Hard stop at home
  }

  public double getPosition() {
    return position;
  }

  public void setPosition(double pos) {
    this.position = pos;
  }
}
