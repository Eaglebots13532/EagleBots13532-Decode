// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.StateMachine;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputStateMachine {
  private Gamepad gamepad1;
  private Gamepad gamepad2;

  // Previous frame state for edge detection -- gamepad1
  private boolean prev1A, prev1B, prev1X, prev1Y;
  private boolean prev1DpadUp, prev1DpadDown, prev1DpadLeft, prev1DpadRight;
  private boolean prev1LeftTrigger, prev1RightTrigger;

  // Previous frame state for edge detection -- gamepad2
  private boolean prev2A, prev2B, prev2X, prev2Y;
  private boolean prev2DpadUp, prev2DpadDown, prev2DpadLeft, prev2DpadRight;
  private boolean prev2LeftTrigger, prev2RightTrigger;

  public interface StateListener {
    void onTogglePrimary(int gamepad, boolean active);

    void onToggleSecondary(int gamepad, boolean active);

    void onActionX(int gamepad);

    void onActionY(int gamepad);

    void onIncrementUp(int gamepad);

    void onIncrementDown(int gamepad);

    void onCycleLeft(int gamepad);

    void onCycleRight(int gamepad);

    void onModifierLeft(int gamepad, float value);

    void onModifierRight(int gamepad, float value);
  }

  private StateListener listener;

  public InputStateMachine(Gamepad gamepad1, Gamepad gamepad2) {
    this.gamepad1 = gamepad1;
    this.gamepad2 = gamepad2;
  }

  public void setListener(StateListener listener) {
    this.listener = listener;
  }

  public void captureInputs() {
    captureGamepad1();
    captureGamepad2();
  }

  public void processState() {
    if (listener == null) return;
    processGamepad1();
    processGamepad2();
  }

  // -----------------------------------------------------------------------
  // Gamepad 1
  // -----------------------------------------------------------------------

  private void captureGamepad1() {
    boolean a = gamepad1.a && !prev1A;
    boolean b = gamepad1.b && !prev1B;
    boolean x = gamepad1.x && !prev1X;
    boolean y = gamepad1.y && !prev1Y;
    boolean du = gamepad1.dpad_up && !prev1DpadUp;
    boolean dd = gamepad1.dpad_down && !prev1DpadDown;
    boolean dl = gamepad1.dpad_left && !prev1DpadLeft;
    boolean dr = gamepad1.dpad_right && !prev1DpadRight;

    prev1A = gamepad1.a;
    prev1B = gamepad1.b;
    prev1X = gamepad1.x;
    prev1Y = gamepad1.y;
    prev1DpadUp = gamepad1.dpad_up;
    prev1DpadDown = gamepad1.dpad_down;
    prev1DpadLeft = gamepad1.dpad_left;
    prev1DpadRight = gamepad1.dpad_right;
    prev1LeftTrigger = gamepad1.left_trigger > 0.1;
    prev1RightTrigger = gamepad1.right_trigger > 0.1;

    // Store edge flags for processState
    gp1Flags =
        new boolean[] {
          a, b, x, y, du, dd, dl, dr, gamepad1.left_trigger > 0.1, gamepad1.right_trigger > 0.1
        };
  }

  private void captureGamepad2() {
    boolean a = gamepad2.a && !prev2A;
    boolean b = gamepad2.b && !prev2B;
    boolean x = gamepad2.x && !prev2X;
    boolean y = gamepad2.y && !prev2Y;
    boolean du = gamepad2.dpad_up && !prev2DpadUp;
    boolean dd = gamepad2.dpad_down && !prev2DpadDown;
    boolean dl = gamepad2.dpad_left && !prev2DpadLeft;
    boolean dr = gamepad2.dpad_right && !prev2DpadRight;

    prev2A = gamepad2.a;
    prev2B = gamepad2.b;
    prev2X = gamepad2.x;
    prev2Y = gamepad2.y;
    prev2DpadUp = gamepad2.dpad_up;
    prev2DpadDown = gamepad2.dpad_down;
    prev2DpadLeft = gamepad2.dpad_left;
    prev2DpadRight = gamepad2.dpad_right;
    prev2LeftTrigger = gamepad2.left_trigger > 0.1;
    prev2RightTrigger = gamepad2.right_trigger > 0.1;

    gp2Flags =
        new boolean[] {
          a, b, x, y, du, dd, dl, dr, gamepad2.left_trigger > 0.1, gamepad2.right_trigger > 0.1
        };
  }

  // Edge flags: [A, B, X, Y, DU, DD, DL, DR, LT, RT]
  private boolean[] gp1Flags = new boolean[10];
  private boolean[] gp2Flags = new boolean[10];

  private void processGamepad1() {
    if (gp1Flags[0]) listener.onTogglePrimary(1, true);
    if (gp1Flags[1]) listener.onToggleSecondary(1, true);
    if (gp1Flags[2]) listener.onActionX(1);
    if (gp1Flags[3]) listener.onActionY(1);
    if (gp1Flags[4]) listener.onIncrementUp(1);
    if (gp1Flags[5]) listener.onIncrementDown(1);
    if (gp1Flags[6]) listener.onCycleLeft(1);
    if (gp1Flags[7]) listener.onCycleRight(1);
    if (gp1Flags[8]) listener.onModifierLeft(1, gamepad1.left_trigger);
    if (gp1Flags[9]) listener.onModifierRight(1, gamepad1.right_trigger);
  }

  private void processGamepad2() {
    if (gp2Flags[0]) listener.onTogglePrimary(2, true);
    if (gp2Flags[1]) listener.onToggleSecondary(2, true);
    if (gp2Flags[2]) listener.onActionX(2);
    if (gp2Flags[3]) listener.onActionY(2);
    if (gp2Flags[4]) listener.onIncrementUp(2);
    if (gp2Flags[5]) listener.onIncrementDown(2);
    if (gp2Flags[6]) listener.onCycleLeft(2);
    if (gp2Flags[7]) listener.onCycleRight(2);
    if (gp2Flags[8]) listener.onModifierLeft(2, gamepad2.left_trigger);
    if (gp2Flags[9]) listener.onModifierRight(2, gamepad2.right_trigger);
  }
}
