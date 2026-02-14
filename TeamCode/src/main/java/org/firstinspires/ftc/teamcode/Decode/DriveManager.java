// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Wraps SwerveDriver and routes inputs based on drive mode.
 *
 * <p>Field-centric: left stick X+Y for translation, right stick X for rotation, wheels steer
 * freely.
 *
 * <p>Tank: wheels lock forward, left stick Y = left wheel, right stick Y = right wheel.
 */
public class DriveManager {

  public enum DriveMode {
    FIELD_CENTRIC,
    TANK
  }

  private final SwerveDriver swerve;
  private final Telemetry telemetry;
  private DriveMode mode = DriveMode.FIELD_CENTRIC;

  public DriveManager(HardwareMap hardwareMap, Telemetry telemetry) {
    this.telemetry = telemetry;
    this.swerve = new SwerveDriver(hardwareMap, telemetry);
  }

  /** Toggle between field-centric and tank drive. */
  public void toggleMode() {
    if (mode == DriveMode.FIELD_CENTRIC) {
      mode = DriveMode.TANK;
    } else {
      mode = DriveMode.FIELD_CENTRIC;
    }
  }

  public void setMode(DriveMode newMode) {
    mode = newMode;
  }

  public DriveMode getMode() {
    return mode;
  }

  /** Reset the IMU heading to zero (re-establish "forward"). */
  public void resetYaw() {
    swerve.resetYaw();
  }

  /**
   * Drive the robot. Pass all four joystick axes (already negated from gamepad).
   *
   * <p>Field-centric uses: leftX, leftY, rightX
   *
   * <p>Tank uses: leftY, rightY
   *
   * @param leftX left stick X (positive = right)
   * @param leftY left stick Y (positive = forward)
   * @param rightX right stick X (positive = rotate CW)
   * @param rightY right stick Y (positive = forward)
   */
  public void drive(double leftX, double leftY, double rightX, double rightY) {
    switch (mode) {
      case FIELD_CENTRIC:
        swerve.swerveDrive(
            leftX * swerve.maxSpeedMetersPerSec,
            leftY * swerve.maxSpeedMetersPerSec,
            rightX * swerve.maxOmegaRadPerSec);
        break;

      case TANK:
        swerve.tankDrive(leftY, rightY);
        break;
    }
  }
}
