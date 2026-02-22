// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.StateMachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivers.AprilDriver;
import org.firstinspires.ftc.teamcode.drivers.ChuteDriver;
import org.firstinspires.ftc.teamcode.drivers.GameDriver;

public class StateMachine extends GameDriver {
  public StateMachine(HardwareMap hardwareMap, Telemetry telemetry) {
    super(hardwareMap, telemetry);
  }

  public AprilDriver april;

  public ChuteDriver chute;

  enum State {
    Intake,
    Approach,
    RangeSet,
    Shoot
  }

  public State currentState = State.Intake;
  public LinearOpMode opMode;
  public Telemetry telemetry;

  public void Init(LinearOpMode opMode) {
    this.opMode = opMode;
    telemetry = opMode.telemetry;
  }

  public void run() {

    switch (currentState) {
      case Intake:
        telemetry.addLine("State Machine: Intake");
        // do stuff in here
        super.intakeOn();
        super.closeGate();

        currentState = State.Approach;
        break;

      case Approach:
        telemetry.addLine("State Machine: Approach");
        // do stuff in here
        super.intakeOff();
        super.closeGate();

        currentState = State.RangeSet;
        break;

      case RangeSet:
        telemetry.addLine("State Machine: RangeSet");
        // do stuff in here
        april.getAprilTag();
        double range = april.getRange();
        super.setLaunchVelocity(range);
        chute.goToPosition(range); // range 0 to 11

        currentState = State.Shoot;
        break;

      case Shoot:
        telemetry.addLine("State Machine: Shoot");
        // do stuff in here
        super.openGate();
        super.intakeOn();
        currentState = State.Intake;
        break;

      default:
        telemetry.addLine("State Machine: error");
        break;
    }
  }
}
