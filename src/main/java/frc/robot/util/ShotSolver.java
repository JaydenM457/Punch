package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.Optional;

public final class ShotSolver {

  private ShotSolver() {}

  public record ShotSolution(
      double initialDistanceMeters,
      double refinedDistanceMeters,
      double shooterRPM,
      double timeOfFlightSeconds,
      Translation2d releasePoseField,
      Translation2d compensatedAimPointField,
      Translation2d launchVectorField,
      Rotation2d correctedHeading) {}

  /**
   * Solves a moving shot using:
   * - predicted release pose
   * - interpolated shooter RPM vs distance
   * - interpolated post-release time of flight vs distance
   * - robot field-relative velocity compensation during note flight
   */
  public static Optional<ShotSolution> solveMovingShot(
      Translation2d robotPoseField,
      Translation2d robotFieldVelocity,
      Translation2d hubCenterField) {

    if (hubCenterField == null) {
      return Optional.empty();
    }

    // Predict robot position when the note actually exits the shooter
    Translation2d releasePose =
        robotPoseField.plus(robotFieldVelocity.times(Constants.RELEASE_LOOKAHEAD_SECS));

    // First pass distance from release pose to hub center
    Translation2d toHubAtRelease = hubCenterField.minus(releasePose);
    double initialDistance = toHubAtRelease.getNorm();

    if (initialDistance < 1e-6) {
      return Optional.empty();
    }

    double initialRPM = Constants.getShotRPM(initialDistance);
    double initialTof = Constants.getShotTimeOfFlight(initialDistance);

    // Compensate only for robot motion during note flight after release
    Translation2d compensatedAimPoint =
        hubCenterField.minus(robotFieldVelocity.times(initialTof));

    // Refine once
    Translation2d refinedLaunchVector = compensatedAimPoint.minus(releasePose);
    double refinedDistance = refinedLaunchVector.getNorm();

    if (refinedDistance < 1e-6) {
      return Optional.empty();
    }

    double refinedRPM = Constants.getShotRPM(refinedDistance);
    double refinedTof = Constants.getShotTimeOfFlight(refinedDistance);

    Translation2d finalAimPoint =
        hubCenterField.minus(robotFieldVelocity.times(refinedTof));

    Translation2d finalLaunchVector = finalAimPoint.minus(releasePose);

    if (finalLaunchVector.getNorm() < 1e-6) {
      return Optional.empty();
    }

    Rotation2d correctedHeading =
        Rotation2d.fromRadians(
            Math.atan2(finalLaunchVector.getY(), finalLaunchVector.getX()));

    return Optional.of(
        new ShotSolution(
            initialDistance,
            refinedDistance,
            refinedRPM,
            refinedTof,
            releasePose,
            finalAimPoint,
            finalLaunchVector,
            correctedHeading));
  }

  /**
   * Stationary/low-motion version. Still uses release lookahead and interpolation,
   * but does not apply flight-time motion compensation.
   */
  public static Optional<ShotSolution> solveStaticShot(
      Translation2d robotPoseField,
      Translation2d robotFieldVelocity,
      Translation2d hubCenterField) {

    if (hubCenterField == null) {
      return Optional.empty();
    }

    Translation2d releasePose =
        robotPoseField.plus(robotFieldVelocity.times(Constants.RELEASE_LOOKAHEAD_SECS));

    Translation2d launchVector = hubCenterField.minus(releasePose);
    double distance = launchVector.getNorm();

    if (distance < 1e-6) {
      return Optional.empty();
    }

    double rpm = Constants.getShotRPM(distance);
    double tof = Constants.getShotTimeOfFlight(distance);

    Rotation2d heading =
        Rotation2d.fromRadians(
            Math.atan2(launchVector.getY(), launchVector.getX()));

    return Optional.of(
        new ShotSolution(
            distance,
            distance,
            rpm,
            tof,
            releasePose,
            hubCenterField,
            launchVector,
            heading));
  }
}