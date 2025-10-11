package org.firstinspires.ftc.teamcode.StateMachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StateMachine {

  enum State {
    Find,
    Approach,
    Aim,
    Shoot
  }
  public State currentState = State.Find;
  public LinearOpMode opMode;
  public Telemetry telemetry;

  public void Init(LinearOpMode opMode) {
    this.opMode = opMode;
    telemetry = opMode.telemetry;
  }

  public void run(){

    switch (currentState) {

      case Find:
        telemetry.addLine("State Machine: Find");
        //do stuff in here

        currentState = State.Approach;
        break;

      case Approach:
        telemetry.addLine("State Machine: Approach");
        //do stuff in here

        currentState = State.Aim;
        break;

      case Aim:
        telemetry.addLine("State Machine: Aim");
        //do stuff in here

        currentState = State.Shoot;
        break;

      case Shoot:
        telemetry.addLine("State Machine: Shoot");
        //do stuff in here

        currentState = State.Find;
        break;

      default:
        telemetry.addLine("State Machine: error");
        break;

    }
  }
}
