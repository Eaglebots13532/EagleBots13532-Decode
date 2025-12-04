package org.firstinspires.ftc.teamcode.Debug;//Notes
//
//• This FTC swerve module implementation provides drive control from -1 to 1
//and steering control for 0-359 degrees
//• Drive Control: The setDrivePower() method accepts power values from -1 to 1,
//with automatic clamping to ensure safe operation
//• Steering Control: Uses a continuous rotation servo for steering with
//proportional control to reach target angles
//• Angle Optimization: Implements shortest path calculation to prevent
//unnecessary 270° rotations when a 90° turn would suffice
//• Encoder Integration: Uses an analog encoder to provide absolute position feedback for the steering mechanism
//• Angle Normalization: All angles are normalized to 0-359 degree range for consistent behavior
//• Hardware Setup: Requires a drive motor (DcMotor), steering servo (CRServo),
//and absolute encoder (AnalogInput) • Usage Example: Call setDriveAndSteer(0.5, 90)
//to drive at half power while turning to 90 degrees
//• The steering uses simple proportional control - you may need to tune the control algorithm based
//on your specific hardware
//
//Code: FTC swerve
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class JimsAI {
  private DcMotor driveMotor;
  private CRServo steerServo;
  private AnalogInput encoder;

  private double targetAngle = 0;
  private double currentAngle = 0;
  private static final double ENCODER_VOLTAGE_RANGE = 3.3;

  public JimsAI(HardwareMap hardwareMap, String driveMotorName, String steerServoName, String encoderName) {
    driveMotor = hardwareMap.get(DcMotor.class, driveMotorName);
    steerServo = hardwareMap.get(CRServo.class, steerServoName);
    encoder = hardwareMap.get(AnalogInput.class, encoderName);

    driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public void setDrivePower(double power) {
    // Clamp drive power between -1 and 1
    power = Math.max(-1.0, Math.min(1.0, power));
    driveMotor.setPower(power);
  }

  public void setTargetAngle(double targetDegrees) {
    // Normalize angle to 0-359 degrees
    targetAngle = normalizeAngle(targetDegrees);
    updateSteering();
  }

  public void setDriveAndSteer(double drivePower, double targetDegrees) {
    setDrivePower(drivePower);
    setTargetAngle(targetDegrees);
  }

  private void updateSteering() {
    currentAngle = getCurrentAngleDegrees();
    double angleDifference = getShortestAngleDifference(currentAngle, targetAngle);

    // Simple proportional control for steering
    double steerPower = angleDifference / 180.0; // Normalize to -1 to 1
    steerPower = Math.max(-1.0, Math.min(1.0, steerPower));

    steerServo.setPower(steerPower);
  }

  private double getCurrentAngleDegrees() {
    double voltage = encoder.getVoltage();
    double angle = (voltage / ENCODER_VOLTAGE_RANGE) * 360.0;
    return angle;
  }

  private double normalizeAngle(double angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
  }

  private double getShortestAngleDifference(double current, double target) {
    double diff = target - current;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
  }

  public double getCurrentAngle() {
    return currentAngle;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public boolean isAtTargetAngle(double tolerance) {
    return Math.abs(getShortestAngleDifference(currentAngle, targetAngle)) < tolerance;
  }
}
