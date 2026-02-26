// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers.wpilib.interpolation;

import org.firstinspires.ftc.teamcode.drivers.wpilib.MathUtil;

/**
 * An inverse interpolation function which determines where within an interpolation range an object
 * lies. This behavior can be linear or nonlinear.
 *
 * @param <T> The type that the {@link InverseInterpolator} will operate on.
 */
@FunctionalInterface
public interface InverseInterpolator<T> {
  /**
   * Return where within interpolation range [0, 1] q is between startValue and endValue.
   *
   * @param startValue Lower part of interpolation range.
   * @param endValue Upper part of interpolation range.
   * @param q Query.
   * @return Interpolant in range [0, 1].
   */
  double inverseInterpolate(T startValue, T endValue, T q);

  /**
   * Returns inverse interpolator for Double.
   *
   * @return Inverse interpolator for Double.
   */
  static InverseInterpolator<Double> forDouble() {
    return MathUtil::inverseInterpolate;
  }
}
