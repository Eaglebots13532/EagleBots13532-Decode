// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.chute;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Example OpMode demonstrating chute control with real hardware.
 *
 * <p>Hardware Config: - Motor: "chute_motor" (DcMotor) - Potentiometer: "chute_pot" (AnalogInput)
 */
@TeleOp(name = "Chute function")
public class ChuteOpMode extends LinearOpMode {

  boolean potage = false;
  double maxPot = 2.0 * Math.PI; // Was 6.16
  double lastVolt = 0.0;
  double home = 0.0; // start voltage
  // does hoodPos = home position
  double hoodPos = 0.0;
  private Chute chute;
  int count = 0;

  // Pot configuration - change this if your pot has different range
  private static final double POT_WRAP_AMOUNT = 6.16;

  // Position presets (in radians)
  private static final double HOME = 0.0;
  private static final double LOW = Math.PI;
  private static final double MID = 2 * Math.PI;
  private static final double HIGH = 3 * Math.PI;
  private static final double MAX = 4 * Math.PI;

  final double STEP_SIZE = 1.8;
  double minChutePos = 0.0;
  boolean homePosSet = false;
  boolean inUpperRegion = false;
  double prevVoltPot = 0.0;
  int majorLoopCt = 0;
  double absChutePos = 0.0;

  // potentiometer voltage change
  double hoodpwr = 0.0;
  double step = 0.0;
  double voltpot = 0.0;
  double correctedChutePos = 0.0;

  private void updatePos(boolean directionUp, double home, FtcPotentiometer pot) {
    double newVoltpot = Math.abs(pot.getVoltage());

    if (directionUp) {
      if (newVoltpot >= maxPot / 2.0) {
        if (!inUpperRegion) {
          inUpperRegion = true;
          majorLoopCt += 1;
        }
      }
      if (newVoltpot < maxPot / 2.0) {
        if (inUpperRegion) {
          inUpperRegion = false;
        }
      }
    } else {
      if (newVoltpot >= maxPot / 2.0) {
        if (!inUpperRegion) {
          inUpperRegion = true;
        }
      }
      if (newVoltpot < maxPot / 2.0) {
        if (inUpperRegion) {
          inUpperRegion = false;
          majorLoopCt -= 1;
        }
      }
    }

    absChutePos = (maxPot * majorLoopCt) + (maxPot - newVoltpot);
    if (!homePosSet) {
      homePosSet = true;
      minChutePos = absChutePos;
    }
    correctedChutePos = absChutePos - minChutePos;

    telemetry.addLine("Potentiometer: " + newVoltpot);
    telemetry.addLine("Loop ct: " + majorLoopCt);
    telemetry.addLine("-- Transition: " + maxPot / 2.0);
    if (inUpperRegion) {
      telemetry.addLine("Region: UPPER");
    } else {
      telemetry.addLine("Region: LOWER");
    }
    telemetry.addLine("Chute POS: " + absChutePos);
    telemetry.addLine("Corrected POS: " + correctedChutePos);
    telemetry.addData("Potentiometer", pot.getVoltage());
    telemetry.update();

    prevVoltPot = newVoltpot;
  }

  void runToHomePos(CRServo chuteMotor, double home, FtcPotentiometer pot) {
    double newVoltpot = Math.abs(pot.getVoltage());
    int stallCount = 0;
    double prevChutePos = 0.0;

    do {
      if (correctedChutePos < 2.0) {
        chuteMotor.setPower(-0.3);
      } else {
        chuteMotor.setPower(-0.8);
      }
      updatePos(false, home, pot);

      // See if we've stalled, if so, increase a count and make sure
      if (correctedChutePos == prevChutePos) {
        stallCount++;
      } else {
        stallCount = 0;
      }

      prevChutePos = correctedChutePos;
    } while (stallCount <= 30); // Loop until we stall at home

    // Reset minChutePos to the new home position
    minChutePos = absChutePos;

    chuteMotor.setPower(0.0);
  }

  @Override
  public void runOpMode() {
    // Get hardware from config
    CRServo chuteMotor = hardwareMap.get(CRServo.class, "chute");
    AnalogInput chutePot = hardwareMap.get(AnalogInput.class, "CP");

    // Create hardware adapters
    FtcCRServo motor = new FtcCRServo(chuteMotor);
    FtcPotentiometer pot = new FtcPotentiometer(chutePot, POT_WRAP_AMOUNT);

    // Create chute controller with real hardware
    ChuteController controller = new ChuteController(motor, pot, MAX);
    chute = new Chute(controller, motor, pot);

    telemetry.addLine("Chute initialized");
    telemetry.addData("Pot wrap", "%.2f rad", POT_WRAP_AMOUNT);
    telemetry.addLine("Press A to home");
    telemetry.update();

    waitForStart();
    if (potage) {
      while (opModeIsActive()) {

        // Update chute (50Hz)
        chute.update(0.02);

        // Control with gamepad
        handleControls();

        // Display status
        updateTelemetry();

        sleep(20); // 50Hz loop
      }
    } else {
      chuteMotor.setPower(hoodpwr);
      for (int i = 0; i < 2; i++) {
        chute.update(0.02);
        home = pot.getVoltage();
        sleep(100);
      }
      lastVolt = home; // sets the present position of pot
      hoodpwr = .8;
      // toggle between booth while loops
      boolean goToTargetPos = true;

      while (opModeIsActive()) {

        if (goToTargetPos) {
          chuteMotor.setPower(0.8);
          updatePos(true, home, pot);

          // FIXME: 11.0 is the max, which can be increased, recommend that this be a constant
          if (correctedChutePos >= 8.0) {
            chuteMotor.setPower(0.0);
            sleep(2000);
            goToTargetPos = false;
          }
        } else {
          // Run chute to home position, this method blocks until chute is at home pos
          // if chute is broken, this will be an infinite loop
          // FIXME: Include a counter to prevent infinite loop
          runToHomePos(chuteMotor, home, pot);
          if (home > 0) chuteMotor.setPower(0.0);
          if (count > 10) chuteMotor.setPower(0.0);
          count = count++;
          telemetry.addData("Power", chuteMotor.getPower());
          telemetry.addData("home", home);
          telemetry.update();
          sleep(500);
          // loop back to top
          goToTargetPos = true;
        } // if else
      } // while
    }
  }

  private void handleControls() {
    // Manual homing
    if (gamepad1.a) {
      chute.home();
    }

    // Emergency stop
    if (gamepad1.b) {
      chute.stop();
    }

    // Position presets
    if (gamepad1.dpad_down) {
      chute.setTargetPosition(HOME);
    }
    if (gamepad1.dpad_left) {
      chute.setTargetPosition(LOW);
    }
    if (gamepad1.dpad_right) {
      chute.setTargetPosition(MID);
    }
    if (gamepad1.dpad_up) {
      chute.setTargetPosition(HIGH);
    }

    // Fine control with triggers
    if (gamepad1.left_trigger > 0.1) {
      double current = chute.getPosition();
      chute.setTargetPosition(current - 0.1);
    }
    if (gamepad1.right_trigger > 0.1) {
      double current = chute.getPosition();
      chute.setTargetPosition(current + 0.1);
    }
  }

  private void updateTelemetry() {
    telemetry.addLine("=== Chute Status ===");
    telemetry.addData("Homed", chute.isHomed() ? "YES" : "NO");
    telemetry.addData("Moving", chute.isMoving() ? "YES" : "NO");
    telemetry.addData(
        "Position", "%.2f rad (%.0f°)", chute.getPosition(), Math.toDegrees(chute.getPosition()));
    if (chute.isHomed()) {
      double percent = (chute.getPosition() / MAX) * 100;
      telemetry.addData("Extension", "%.0f%%", percent);
    }
    telemetry.addData("poteniomter voltage:", chute.getPot());

    telemetry.addLine();
    telemetry.addLine("=== Controls ===");
    telemetry.addLine("A: Home | B: Stop");
    telemetry.addLine("D-Pad: Presets");
    telemetry.addLine("Triggers: Fine adjust");

    telemetry.update();
  }
}
