package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

public class Swerve {

  LinearOpMode opMode;
  public DcMotor brMotor, blMotor, frMotor, flMotor;
  public Servo brServo, blServo, frServo, flServo;
  private final Telemetry telemetry;
  public double robot_Angle;
  public final GoBildaPinpointDriver odo;

  public Swerve(LinearOpMode opMode, GoBildaPinpointDriver odo) {
    this.opMode = opMode;
    telemetry = opMode.telemetry;
    // might change odo to keep it isolated in here that way there aren't conflicting odo's due to odo being in pivot as well
    this.odo = odo;
  }

  public void drive(double x, double y, double rotate) {
    odo.update();

    double vector_ang = Math.atan2(x, y) * (180 / Math.PI);
    if(vector_ang < 0)
      vector_ang = 360 + vector_ang;
    double vector_pow = Math.sqrt((x * x + y * y));
    telemetry.addLine("input vector angle: " + vector_ang);
    telemetry.addLine("input vector power: " + vector_pow);
    vector_ang = (vector_ang - 90) % 360;
    vector_ang /= 360;
    telemetry.addLine("input vector angle: " + vector_ang);

    //0/360 in front, 90 at right, 180 back, and 270 left
  }
}
