// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import frc.robot.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {
  private static final String TABLE_KEY = "TunableNumbers";

  private final String KEY;
  private boolean _hasDefault = false;
  private double _defaultValue;
  private LoggedDashboardNumber _dashboardNumber;
  private Map<Integer, Double> _lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.KEY = TABLE_KEY + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!_hasDefault) {
      _hasDefault = true;
      this._defaultValue = defaultValue;
      if (Constants.TUNING_MODE) {
        _dashboardNumber = new LoggedDashboardNumber(KEY, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!_hasDefault) {
      return 0.0;
    } else {
      return Constants.TUNING_MODE ? _dashboardNumber.get() : _defaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = _lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      _lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }
}