// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers.wpilib.interpolation;

import java.util.Map;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined by making a guess
 * from points that are defined. This uses linear interpolation.
 *
 * <p>Example of use:
 *
 * <pre><code>
 * InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
 * table.put(0.0, 0.0);
 * table.put(1.0, 10.0);
 * table.put(2.0, 30.0);
 * // ...
 * double result = table.get(1.5); // Returns 20.0
 * </code></pre>
 */
public class InterpolatingDoubleTreeMap extends InterpolatingTreeMap<Double, Double> {
  /** Default constructor. */
  public InterpolatingDoubleTreeMap() {
    super(InverseInterpolator.forDouble(), Interpolator.forDouble());
  }

  /**
   * Creates an {@link InterpolatingDoubleTreeMap} from the given entries.
   *
   * @param entries The entries to add to the map.
   * @return The map filled with the {@code entries}.
   */
  @SafeVarargs
  public static InterpolatingDoubleTreeMap ofEntries(Map.Entry<Double, Double>... entries) {
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    for (var entry : entries) {
      map.put(entry.getKey(), entry.getValue());
    }
    return map;
  }
}
