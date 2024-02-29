package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// NOTE: This file is available at
// https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d

public class LocalADStarAK implements Pathfinder {
  private final ADStarIO _io = new ADStarIO();

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  @Override
  public boolean isNewPathAvailable() {
    if (!Logger.hasReplaySource()) {
      _io.updateIsNewPathAvailable();
    }

    Logger.processInputs("LocalADStarAK", _io);

    return _io._isNewPathAvailable;
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    if (!Logger.hasReplaySource()) {
      _io.updateCurrentPathPoints(constraints, goalEndState);
    }

    Logger.processInputs("LocalADStarAK", _io);

    if (_io._currentPathPoints.isEmpty()) {
      return null;
    }

    return PathPlannerPath.fromPathPoints(_io._currentPathPoints, constraints, goalEndState);
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (!Logger.hasReplaySource()) {
      _io._adStar.setStartPosition(startPosition);
    }
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (!Logger.hasReplaySource()) {
      _io._adStar.setGoalPosition(goalPosition);
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path to properly avoid obstacles
   */
  @Override
  public void setDynamicObstacles(
    List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    if(!Logger.hasReplaySource()){
      _io._adStar.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  private static class ADStarIO implements LoggableInputs {
    public LocalADStar _adStar = new LocalADStar();
    public boolean _isNewPathAvailable = false;
    public List<PathPoint> _currentPathPoints = Collections.emptyList();

    @Override
    public void toLog(LogTable table) {
      table.put("IsNewPathAvailable", _isNewPathAvailable);

      double[] pointsLogged = new double[_currentPathPoints.size() * 2];
      int idx = 0;
      for (PathPoint point : _currentPathPoints) {
        pointsLogged[idx] = point.position.getX();
        pointsLogged[idx + 1] = point.position.getY();
        idx += 2;
      }

      table.put("CurrentPathPoints", pointsLogged);
    }

    @Override
    public void fromLog(LogTable table) {
      _isNewPathAvailable = table.get("IsNewPathAvailable", false);

      double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);

      List<PathPoint> pathPoints = new ArrayList<>();
      for (int i = 0; i < pointsLogged.length; i += 2) {
        pathPoints.add(
            new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
      }

      _currentPathPoints = pathPoints;
    }

    public void updateIsNewPathAvailable() {
      _isNewPathAvailable = _adStar.isNewPathAvailable();
    }

    public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
      PathPlannerPath currentPath = _adStar.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        _currentPathPoints = currentPath.getAllPathPoints();
      } else {
        _currentPathPoints = Collections.emptyList();
      }
    }
  }
}
