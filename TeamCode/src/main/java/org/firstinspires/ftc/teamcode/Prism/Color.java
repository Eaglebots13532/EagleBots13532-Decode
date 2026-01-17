// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Prism;

public class Color {
  public int red;
  public int green;
  public int blue;

  public Color(int red, int green, int blue) {
    this.red = Math.min(red, 255);
    this.green = Math.min(green, 255);
    this.blue = Math.min(blue, 255);
  }

  @Override
  public String toString() {
    return String.format("%d, %d, %d", red, green, blue);
  }

  public static final Color RED = new Color(255, 0, 0);
  public static final Color ORANGE = new Color(255, 165, 0);
  public static final Color YELLOW = new Color(255, 255, 0);
  public static final Color OLIVE = new Color(128, 128, 0);
  public static final Color GREEN = new Color(0, 255, 0);
  public static final Color CYAN = new Color(0, 255, 255);
  public static final Color BLUE = new Color(0, 0, 255);
  public static final Color TEAL = new Color(0, 128, 128);
  public static final Color MAGENTA = new Color(255, 0, 255);
  public static final Color PURPLE = new Color(128, 0, 128);
  public static final Color PINK = new Color(255, 20, 128);
  public static final Color WHITE = new Color(255, 255, 255);
  public static final Color TRANSPARENT = new Color(0, 0, 0);
}
