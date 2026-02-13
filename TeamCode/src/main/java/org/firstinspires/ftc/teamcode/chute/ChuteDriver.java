// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.chute;

import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Decode.chute.FtcPotentiometer;

public class ChuteDriver {

  // --- Safety limits ---
  private static final double MAX_CHUTE_POS = 11.0;
  private static final int HOME_TIMEOUT_MS = 5000;

  // --- Position presets ---
  public static final double POS_HOME = 0.0;
  public static final double POS_LOW = 3.0;
  public static final double POS_MID = 8.0;
  public static final double POS_HIGH = MAX_CHUTE_POS;

  // --- Pot config ---
  private double maxPot = 2.0 * Math.PI;

  // --- Hardware ---
  private final CRServo chuteMotor;
  private final FtcPotentiometer pot;
  private final Telemetry telemetry;

  // --- State tracking ---
  private double minChutePos = 0.0;
  private boolean homePosSet = false;
  private boolean inUpperRegion = false;
  private double prevVoltPot = 0.0;
  private int majorLoopCt = 0;
  private double absChutePos = 0.0;
  private double correctedChutePos = 0.0;

  // --- Command state ---
  private enum Mode {
    IDLE,
    MOVING_TO_TARGET,
    HOMING
  }

  private Mode mode = Mode.IDLE;
  private double targetPos = 0.0;
  private long homeStartTime = 0;
  private int stallCount = 0;
  private double prevHomingPos = 0.0;
  private boolean homed = false;

  // -----------------------------------------------------------------------
  // Completion callbacks
  // -----------------------------------------------------------------------

  public interface ChuteListener {
    /** Fired when the chute reaches its commanded target position. */
    void onTargetReached(double position);

    /** Fired when the chute successfully homes via stall detection. */
    void onHomeComplete();

    /** Fired when homing times out -- chute may not be at home. */
    void onHomeTimeout();

    /** Fired when the chute is stopped via stop(). */
    void onStopped();
  }

  private ChuteListener listener;

  public ChuteDriver(CRServo chuteMotor, FtcPotentiometer pot, Telemetry telemetry) {
    this.chuteMotor = chuteMotor;
    this.pot = pot;
    this.telemetry = telemetry;
  }

  public void setListener(ChuteListener listener) {
    this.listener = listener;
  }

  // -----------------------------------------------------------------------
  // Public commands -- call these from your InputStateMachine callbacks
  // -----------------------------------------------------------------------

  /** Move chute to a target position (clamped to MAX_CHUTE_POS). */
  public void goToPosition(double position) {
    targetPos = Math.min(Math.max(position, 0.0), MAX_CHUTE_POS);
    if (targetPos < 0.001) {
      goHome();
    } else {
      mode = Mode.MOVING_TO_TARGET;
    }
  }

  /** Run chute to home position (stall-detect with timeout). */
  public void goHome() {
    mode = Mode.HOMING;
    stallCount = 0;
    prevHomingPos = 0.0;
    homeStartTime = System.currentTimeMillis();
  }

  /** Immediately stop the motor and cancel any active command. */
  public void stop() {
    mode = Mode.IDLE;
    chuteMotor.setPower(0.0);
    if (listener != null) listener.onStopped();
  }

  /** Returns true if the chute is actively moving toward a target or homing. */
  public boolean isBusy() {
    return mode != Mode.IDLE;
  }

  /** Returns true if the chute has successfully homed at least once. */
  public boolean isHomed() {
    return homed;
  }

  /** Returns the current corrected chute position. */
  public double getPosition() {
    return correctedChutePos;
  }

  // -----------------------------------------------------------------------
  // Call this every loop cycle (~50Hz)
  // -----------------------------------------------------------------------
  public void update() {
    switch (mode) {
      case MOVING_TO_TARGET:
        updateMovingToTarget();
        break;
      case HOMING:
        updateHoming();
        break;
      case IDLE:
      default:
        break;
    }

    telemetry.addData("Chute Mode", mode);
    telemetry.addData("Chute Pos", "%.2f", correctedChutePos);
    telemetry.addData("Chute Target", "%.2f", targetPos);
  }

  // -----------------------------------------------------------------------
  // Internal state machine
  // -----------------------------------------------------------------------

  private void updateMovingToTarget() {
    updatePos(true);

    if (correctedChutePos >= targetPos) {
      chuteMotor.setPower(0.0);
      mode = Mode.IDLE;
      if (listener != null) listener.onTargetReached(correctedChutePos);
    } else {
      chuteMotor.setPower(0.8);
    }
  }

  private void updateHoming() {
    // Timeout safety
    if (System.currentTimeMillis() - homeStartTime > HOME_TIMEOUT_MS) {
      chuteMotor.setPower(0.0);
      telemetry.addLine("HOME TIMEOUT - stopped");
      mode = Mode.IDLE;
      if (listener != null) listener.onHomeTimeout();
      return;
    }

    updatePos(false);

    // Slow down when close
    if (correctedChutePos < 2.0) {
      chuteMotor.setPower(-0.3);
    } else {
      chuteMotor.setPower(-0.8);
    }

    // Stall detection
    if ((int) correctedChutePos == (int) prevHomingPos) {
      stallCount++;
    } else {
      stallCount = 0;
    }
    prevHomingPos = correctedChutePos;

    if (stallCount > 50) {
      chuteMotor.setPower(0.0);
      minChutePos = absChutePos;
      homed = true;
      mode = Mode.IDLE;
      telemetry.addLine("Found home");
      if (listener != null) listener.onHomeComplete();
    }
  }

  // -----------------------------------------------------------------------
  // Position tracking (from potentiometer)
  // -----------------------------------------------------------------------
  private void updatePos(boolean directionUp) {
    double newVoltpot = Math.abs(pot.getVoltage());

    if (directionUp) {
      if (newVoltpot >= maxPot / 2.0 && !inUpperRegion) {
        inUpperRegion = true;
        majorLoopCt++;
      }
      if (newVoltpot < maxPot / 2.0 && inUpperRegion) {
        inUpperRegion = false;
      }
    } else {
      if (newVoltpot >= maxPot / 2.0 && !inUpperRegion) {
        inUpperRegion = true;
      }
      if (newVoltpot < maxPot / 2.0 && inUpperRegion) {
        inUpperRegion = false;
        majorLoopCt--;
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
}
