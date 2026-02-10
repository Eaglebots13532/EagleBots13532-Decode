// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.chute.Chute;
import org.firstinspires.ftc.teamcode.chute.ChuteController;
import org.firstinspires.ftc.teamcode.chute.FtcCRServo;
import org.firstinspires.ftc.teamcode.chute.FtcPotentiometer;
import org.firstinspires.ftc.teamcode.chute.MockMotor;
import org.firstinspires.ftc.teamcode.chute.MockPotentiometer;

/**
 * Example OpMode demonstrating chute control with real hardware.
 *
 * <p>Hardware Config: - Motor: "chute_motor" (DcMotor) - Potentiometer: "chute_pot" (AnalogInput)
 */
public class DC_chuteControl {
  private LinearOpMode myOp = null;

  // Default constructor
  public DC_chuteControl(LinearOpMode opmode) {
    myOp = opmode;
  }

  boolean potage = false;
  double maxPot = 2.0 * Math.PI; // Was 6.16
  double lastVolt = 0.0;
  double home = 0.0; // start voltage
  // does hoodPos = home position
  double hoodPos = 0.0;
  private Chute chute;

  // Pot configuration - change this if your pot has different range
  private static final double POT_WRAP_AMOUNT = 6.16;

  // Position presets (in radians)
  private static final double HOME = 0.0;
  private static final double LOW = Math.PI;
  private static final double MID = Math.PI;
  private static final double HIGH = Math.PI;
  private static final double MAX = 2 * Math.PI;

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
  boolean goToTargetPos = true;

  private void updatePos(boolean directionUp, double home, MockPotentiometer pot) {
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

    prevVoltPot = newVoltpot;
  }

  void runToHomePos(MockMotor chuteMotor, double home, MockPotentiometer pot) {
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

  public void initHood() {
    // Get hardware from config
    FtcCRServo motor = new FtcCRServo(myOp.hardwareMap.get(CRServo.class, "chute"));
    FtcPotentiometer pot = new FtcPotentiometer(myOp.hardwareMap.get(AnalogInput.class, "CP"));
    ChuteController controller = new ChuteController(motor, pot, 4 * Math.PI);
    Chute chute = new Chute(controller, motor, pot);
  }

  public void setHome() {
    chute.update(0.02);
    chute.isHomed();
  }

  public void HoodPosition(double target) {

    chute.update(0.02);
    chute.setTargetPosition(target);
    // if not at target moving
    chute.update(.02);
    chute.getPosition();
    myOp.telemetry.addData("target", chute.isAtTarget());
    myOp.telemetry.addData("stopping", chute.getPosition());
    chute.stop();
  }

  public void hoodStop() {
    chute.update(0.02);
    chute.stop();
  }

  public void HoodHome() {
    runToHomePos(chute.getMotor(), home, chute.getPot());
    myOp.sleep(500);
  }
} // end DC chute control
