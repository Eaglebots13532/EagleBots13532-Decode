// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.StateMachine;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputStateMachine {
  private Gamepad gamepad1;
  private Gamepad gamepad2;

  // Previous frame state for edge detection
  private boolean prevA, prevB, prevX, prevY;
  private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
  private boolean prevLeftTrigger, prevRightTrigger;

  // Action flags
  public boolean togglePrimary = false; // A
  public boolean toggleSecondary = false; // B
  public boolean actionX = false; // X
  public boolean actionY = false; // Y
  public boolean incrementUp = false; // dpad_up
  public boolean incrementDown = false; // dpad_down
  public boolean cycleLeft = false; // dpad_left
  public boolean cycleRight = false; // dpad_right
  public boolean modifierLeft = false; // left_trigger
  public boolean modifierRight = false; // right_trigger

  public interface StateListener {
    void onTogglePrimary(boolean active);

    void onToggleSecondary(boolean active);

    void onActionX();

    void onActionY();

    void onIncrementUp();

    void onIncrementDown();

    void onCycleLeft();

    void onCycleRight();

    void onModifierLeft(float value);

    void onModifierRight(float value);
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
    // Rising edge detection -- flag only on the frame the button is first pressed
    togglePrimary = gamepad1.a && !prevA;
    toggleSecondary = gamepad1.b && !prevB;
    actionX = gamepad1.x && !prevX;
    actionY = gamepad1.y && !prevY;
    incrementUp = gamepad1.dpad_up && !prevDpadUp;
    incrementDown = gamepad1.dpad_down && !prevDpadDown;
    cycleLeft = gamepad1.dpad_left && !prevDpadLeft;
    cycleRight = gamepad1.dpad_right && !prevDpadRight;
    modifierLeft = gamepad1.left_trigger > 0.1;
    modifierRight = gamepad1.right_trigger > 0.1;

    // Store current state for next frame
    prevA = gamepad1.a;
    prevB = gamepad1.b;
    prevX = gamepad1.x;
    prevY = gamepad1.y;
    prevDpadUp = gamepad1.dpad_up;
    prevDpadDown = gamepad1.dpad_down;
    prevDpadLeft = gamepad1.dpad_left;
    prevDpadRight = gamepad1.dpad_right;
    prevLeftTrigger = modifierLeft;
    prevRightTrigger = modifierRight;
  }

  public void processState() {
    if (listener == null) return;

    if (togglePrimary) listener.onTogglePrimary(true);
    if (toggleSecondary) listener.onToggleSecondary(true);
    if (actionX) listener.onActionX();
    if (actionY) listener.onActionY();
    if (incrementUp) listener.onIncrementUp();
    if (incrementDown) listener.onIncrementDown();
    if (cycleLeft) listener.onCycleLeft();
    if (cycleRight) listener.onCycleRight();
    if (modifierLeft) listener.onModifierLeft(gamepad1.left_trigger);
    if (modifierRight) listener.onModifierRight(gamepad1.right_trigger);
  }
}
