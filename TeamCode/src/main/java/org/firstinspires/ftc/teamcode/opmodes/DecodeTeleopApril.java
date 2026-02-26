// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.StateMachine.StateMachine.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.StateMachine.InputStateMachine;
import org.firstinspires.ftc.teamcode.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.drivers.AprilDriver;
import org.firstinspires.ftc.teamcode.drivers.ChuteDriver;
import org.firstinspires.ftc.teamcode.drivers.DriveManager;
import org.firstinspires.ftc.teamcode.drivers.GameDriver;
import org.firstinspires.ftc.teamcode.drivers.chute.FtcPotentiometer;

@TeleOp(name = "April Teleop")
public class DecodeTeleopApril extends LinearOpMode {

  private static final double CHUTE_STEP = 3.0;
  private static final double POT_WRAP_AMOUNT = 6.16;

  private boolean chuteInputsLocked = false;
  private boolean gateOpen = false;
  private boolean autoRange = false;

  // private double flywheelPower = 0.0;
  private double flywheelPower = 1850.0;
    double flyvelocity = 1500;
    boolean istag = true;
    double range = 90; // average distance

  @Override
  public void runOpMode() {
    // --- Subsystems ---
    AprilDriver april = new AprilDriver(this);
    april.initAprilTag(); // initialize camera to read april tags
    DriveManager driveManager = new DriveManager(this, hardwareMap, telemetry);
    GameDriver game = new GameDriver(hardwareMap, telemetry);
    StateMachine LaunchCtl = new StateMachine(hardwareMap, telemetry);
    CRServo chuteMotor = hardwareMap.get(CRServo.class, "chute");
    AnalogInput chutePot = hardwareMap.get(AnalogInput.class, "CP");
    FtcPotentiometer pot = new FtcPotentiometer(chutePot, POT_WRAP_AMOUNT);
    ChuteDriver chute = new ChuteDriver(chuteMotor, pot, telemetry);
    game.intakeOn(); // leave the intake on

    // --- Input ---
    InputStateMachine sm = new InputStateMachine(gamepad1, gamepad2);

    // --- Chute completion callbacks ---
    chute.setListener(
        new ChuteDriver.ChuteListener() {
          @Override
          public void onTargetReached(double position) {
            chuteInputsLocked = false;
            // telemetry.addLine("Chute reached: " +  managerposition);
          }

          @Override
          public void onHomeComplete() {
            chuteInputsLocked = false;
            telemetry.addLine("Chute homed");
          }

          @Override
          public void onHomeTimeout() {
            chuteInputsLocked = false;
            telemetry.addLine("WARNING: Chute home timed out");
          }

          @Override
          public void onStopped() {
            chuteInputsLocked = false;
            telemetry.addLine("Chute stopped");
          }
        });

    // --- Input callbacks ---
    sm.setListener(
        new InputStateMachine.StateListener() {
          // A button -- gamepad2: toggle gate
          @Override
          public void onTogglePrimary(int gamepad, boolean active) {
            if (gamepad == 2) {
              game.closeGate();
              // game.toggleIntake();
            }
          }

          // B button -- gamepad2: toggle intake
          @Override
          public void onToggleSecondary(int gamepad, boolean active) {
            if (gamepad == 2) {
                gateOpen = !gateOpen;
              // game.toggleIntake();
              if(gateOpen) game.openGate(); else game.closeGate();

            }
          }

          // X button -- gamepad2: send chute home
          @Override
          public void onActionX(int gamepad) {
            if (gamepad == 2) {
                autoRange = !autoRange;
                if(autoRange) {
                    telemetry.addLine("Auto Range On");
                } // auto one
                else {
                    telemetry.addLine("Auto Range off");
                }
            } // end if gamepad = 2
          } // end onActionX

          // Y button -- gamepad2: emergency stop chute
          @Override
          public void onActionY(int gamepad) {
            if (gamepad == 2) {
              chute.stop();
            }
          }

          // Dpad right -- gamepad2: extend chute
          @Override
          public void onCycleRight(int gamepad) {
            if (gamepad == 2 && !chuteInputsLocked) {
              chuteInputsLocked = true;
              chute.goToPosition(chute.getPosition() + CHUTE_STEP);
            }
          }

          // Dpad left -- gamepad2: retract chute
          @Override
          public void onCycleLeft(int gamepad) {
            if (gamepad == 2 && !chuteInputsLocked) {
              chuteInputsLocked = true;
              double target = chute.getPosition() - CHUTE_STEP;
              if (target < 0.001) {
                chute.goHome();
              } else {
                chute.goToPosition(target);
              }
            }
          }

          // Dpad down -- gamepad1: toggle drive mode
          @Override
          public void onIncrementDown(int gamepad) {
            if (gamepad == 1) {
              driveManager.toggleMode();
            } else if (gamepad == 2) {
              flywheelPower -= 50.0;
              if (flywheelPower < 1800.0) {
                flywheelPower = 1800.0;
              }
              // game.setLaunchPower(flywheelPower);
              game.setLaunchVelocity(flywheelPower);
            }
          }

          @Override
          public void onIncrementUp(int gamepad) {
            if (gamepad == 2) {
              flywheelPower += 50.0;
              if (flywheelPower > 2100) {
                flywheelPower = 2100;
              }
              // game.setLaunchPower(flywheelPower);
              game.setLaunchVelocity(flywheelPower);
            }
          }

          // Left trigger -- gamepad2: Hood positon (proportional)
          @Override
          public void onModifierLeft(int gamepad, float value) {
            final double hoodMax = 11.0;
            if (gamepad == 2) {
              game.setIntakePower(value);
            }
          }

          // Right trigger -- gamepad2: flywheel speed (proportional)
          @Override
          public void onModifierRight(int gamepad, float value) {
            if (gamepad == 2) {
              game.setLaunchPower(value);
            }
          }
        });

    telemetry.addLine("Initialized -- waiting for start");
    telemetry.addData("Drive Mode", driveManager.getMode());
    telemetry.update();

    waitForStart();

    // --- Main loop ---
    while (opModeIsActive()) {
      april.getAprilTag();
      telemetry.addData("April range", april.getRange());
      telemetry.addData("April tag", april.getMetaId());
      telemetry.update();
      if (!chute.isHomed()) {
        telemetry.addLine("Homing chute...");
        telemetry.update();
        chute.goHome();
        while (opModeIsActive() && chute.isBusy()) {
          chute.update();
          telemetry.update();
          sleep(20);
        }
      }
      if(autoRange){
          if(istag){
              april.getAprilTag();// get tag data
              range = april.getRange();
              int tag = april.getMetaId();
              istag = tag != 20 || tag != 24;
              if (istag) {
                  // in teleOp we are facing the correct april tag
                  // at present there is no check for match tag
                  flyvelocity = (1.6374 * range) + 1510;
                  game.setLaunchVelocity(flyvelocity);
                  // check hood position
              }
          } else {
              // the robot is close for launching set default 1500
              game.setLaunchVelocity(flyvelocity);
          }
          // check hood position
          // range loses scope so initialize range or use default
          if (istag) range = april.getRange();
          if (range <= 50) {
              if (chute.getPosition() < 3.0) {
                  chute.goHome();
                  sleep(500);
                  chute.goToPosition(3.0);
              }
          } else if (range <= 90) {
              if (chute.getPosition() < 5.0) {
                  chute.goHome();
                  sleep(600);
                  chute.goToPosition(6.0);
              } else if (range <= 150) {
                  if (chute.getPosition() < 7.0) {
                      chute.goHome();
                      sleep(700);
                      chute.goToPosition(7.0);
                  }
              }
          }
      }
      // Discrete button events
      sm.captureInputs();
      sm.processState();

      // Gamepad1: drive (polled)
      driveManager.drive(
          -gamepad1.left_stick_x,
          -gamepad1.left_stick_y,
          -gamepad1.right_stick_x,
          -gamepad1.right_stick_y);
      // Gamepad2: arm and tilt (polled, continuous)
      game.setArmPower(-gamepad2.right_stick_y);
      game.setTilt((-gamepad2.left_stick_y + 1.0) / 2.0); // map -1..1 to 0..1

      // Subsystem updates
      chute.update();
      // game.updateTelemetry();
      // telemetry.update();
      sleep(20);
    }
  }
}
