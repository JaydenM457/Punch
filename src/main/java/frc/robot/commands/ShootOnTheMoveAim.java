package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsytem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class ShootOnTheMoveAim extends Command {
  private final SwerveSubsystem swerve;
  private final Vision vision;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final HopperSubsytem hopper;

  private final DoubleSupplier xInput;
  private final DoubleSupplier yInput;
  private final BooleanSupplier feedButton;

  private final PIDController thetaPid =
      new PIDController(
          Constants.HUB_THETA_KP,
          Constants.HUB_THETA_KI,
          Constants.HUB_THETA_KD);

  private int lockedTagId = -1;
  private Translation2d hubCenterField = null;
  private boolean seeingTag = false;

  public ShootOnTheMoveAim(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      HopperSubsytem hopper,
      DoubleSupplier xInput,
      DoubleSupplier yInput,
      BooleanSupplier feedButton) {
    this.swerve = swerve;
    this.vision = swerve.getVision();
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.xInput = xInput;
    this.yInput = yInput;
    this.feedButton = feedButton;

    thetaPid.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, shooter, indexer, hopper);
  }

  @Override
  public void initialize() {
    lockedTagId = -1;
    seeingTag = false;
    thetaPid.reset();

    hubCenterField =
        vision.getHubCenterFieldPositionFromLayout(Constants.HUB_TAG_IDS).orElse(null);
  }

  @Override
  public void execute() {
    Optional<Integer> visibleTag = vision.getBestVisibleHubTag(Constants.HUB_TAG_IDS);
    seeingTag = visibleTag.isPresent();

    if (visibleTag.isPresent()) {
      lockedTagId = visibleTag.get();
    }

    if (lockedTagId != -1) {
      vision.getHubCenterFieldPositionFromTag(lockedTagId)
          .ifPresent(center -> hubCenterField = center);
    } else {
      vision.getHubCenterFieldPositionFromLayout(Constants.HUB_TAG_IDS)
          .ifPresent(center -> hubCenterField = center);
    }

    Pose2d robotPose = swerve.getPose();

    // Driver controls translation only
    ChassisSpeeds commandedFieldVelocity =
        swerve.getTargetSpeeds(
            xInput.getAsDouble(),
            yInput.getAsDouble(),
            robotPose.getRotation());

    ChassisSpeeds measuredFieldVelocity = swerve.getFieldVelocity();

    Translation2d commandedVel =
        new Translation2d(
            commandedFieldVelocity.vxMetersPerSecond,
            commandedFieldVelocity.vyMetersPerSecond);

    Translation2d measuredVel =
        new Translation2d(
            measuredFieldVelocity.vxMetersPerSecond,
            measuredFieldVelocity.vyMetersPerSecond);

    // Blend driver intent and actual robot motion
    Translation2d estimatedVel =
        measuredVel.times(Constants.MEASURED_VELOCITY_WEIGHT)
            .plus(commandedVel.times(1.0 - Constants.MEASURED_VELOCITY_WEIGHT));

    // Predict robot pose when the note actually leaves the shooter
    Translation2d releasePose =
        robotPose.getTranslation().plus(estimatedVel.times(Constants.RELEASE_LOOKAHEAD_SECS));

    if (hubCenterField == null) {
      swerve.driveFieldOriented(
          new ChassisSpeeds(
              commandedFieldVelocity.vxMetersPerSecond,
              commandedFieldVelocity.vyMetersPerSecond,
              0.0));
      shooter.set(0.0);
      indexer.setduty(0.0);
      hopper.setduty(0.0);
      return;
    }

    // Distance from release pose to hub center
    Translation2d toHubAtRelease = hubCenterField.minus(releasePose);
    double distanceMeters = toHubAtRelease.getNorm();

    // First pass from interpolation maps
    double shooterRPM = Constants.getShotRPM(distanceMeters);
    double timeOfFlight = Constants.getShotTimeOfFlight(distanceMeters);

    // Compensate for robot motion ONLY during note flight after release
    Translation2d compensatedAimPoint =
        hubCenterField.minus(estimatedVel.times(timeOfFlight));

    // Refine once for better accuracy
    Translation2d refinedVector = compensatedAimPoint.minus(releasePose);
    double refinedDistance = refinedVector.getNorm();

    double refinedRPM = Constants.getShotRPM(refinedDistance);
    double refinedTof = Constants.getShotTimeOfFlight(refinedDistance);

    Translation2d finalAimPoint =
        hubCenterField.minus(estimatedVel.times(refinedTof));

    Translation2d finalLaunchVector =
        finalAimPoint.minus(releasePose);

    Rotation2d correctedHeading =
        Rotation2d.fromRadians(
            Math.atan2(finalLaunchVector.getY(), finalLaunchVector.getX()));

    shooter.setMechanismVelocitySetpoint(RPM.of(refinedRPM));

    double omega =
        thetaPid.calculate(
            robotPose.getRotation().getRadians(),
            correctedHeading.getRadians());

    omega =
        MathUtil.clamp(
            omega,
            -Constants.HUB_MAX_OMEGA_RAD_PER_SEC,
            Constants.HUB_MAX_OMEGA_RAD_PER_SEC);

    // Driver owns X/Y, command owns theta
    swerve.driveFieldOriented(
        new ChassisSpeeds(
            commandedFieldVelocity.vxMetersPerSecond,
            commandedFieldVelocity.vyMetersPerSecond,
            omega));

    double headingErrorDeg =
        Math.abs(robotPose.getRotation().minus(correctedHeading).getDegrees());
    if (headingErrorDeg > 180.0) {
      headingErrorDeg = 360.0 - headingErrorDeg;
    }

    double headingToleranceDeg =
        seeingTag
            ? Constants.HUB_HEADING_TOLERANCE_DEG_WITH_TAG
            : Constants.HUB_HEADING_TOLERANCE_DEG_NO_TAG;

    boolean drivetrainReady = headingErrorDeg <= headingToleranceDeg;
    boolean shooterReady =
        shooter.getVelocity().in(RPM) >= refinedRPM * Constants.SHOOTER_READY_FRACTION;

    boolean shouldFeed =
        feedButton.getAsBoolean() && drivetrainReady && shooterReady;

    if (shouldFeed) {
      indexer.setduty(-1.0);
      hopper.setduty(-1.0);
    } else {
      indexer.setduty(0.0);
      hopper.setduty(0.0);
    }

    SmartDashboard.putBoolean("SOTM_PID/SeeingTag", seeingTag);
    SmartDashboard.putNumber("SOTM_PID/LockedTagId", lockedTagId);
    SmartDashboard.putNumber("SOTM_PID/HubCenterX", hubCenterField.getX());
    SmartDashboard.putNumber("SOTM_PID/HubCenterY", hubCenterField.getY());
    SmartDashboard.putNumber("SOTM_PID/ReleasePoseX", releasePose.getX());
    SmartDashboard.putNumber("SOTM_PID/ReleasePoseY", releasePose.getY());
    SmartDashboard.putNumber("SOTM_PID/DistanceMeters", distanceMeters);
    SmartDashboard.putNumber("SOTM_PID/RefinedDistanceMeters", refinedDistance);
    SmartDashboard.putNumber("SOTM_PID/TOF", refinedTof);
    SmartDashboard.putNumber("SOTM_PID/ShooterTargetRPM", refinedRPM);
    SmartDashboard.putNumber("SOTM_PID/CorrectedHeadingDeg", correctedHeading.getDegrees());
    SmartDashboard.putNumber("SOTM_PID/HeadingErrorDeg", headingErrorDeg);
    SmartDashboard.putNumber("SOTM_PID/CommandedVX", commandedVel.getX());
    SmartDashboard.putNumber("SOTM_PID/CommandedVY", commandedVel.getY());
    SmartDashboard.putNumber("SOTM_PID/MeasuredVX", measuredVel.getX());
    SmartDashboard.putNumber("SOTM_PID/MeasuredVY", measuredVel.getY());
    SmartDashboard.putNumber("SOTM_PID/EstimatedVX", estimatedVel.getX());
    SmartDashboard.putNumber("SOTM_PID/EstimatedVY", estimatedVel.getY());
    SmartDashboard.putBoolean("SOTM_PID/DrivetrainReady", drivetrainReady);
    SmartDashboard.putBoolean("SOTM_PID/ShooterReady", shooterReady);
    SmartDashboard.putBoolean("SOTM_PID/FeedButton", feedButton.getAsBoolean());
    SmartDashboard.putBoolean("SOTM_PID/Feeding", shouldFeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.set(0.0);
    indexer.setduty(0.0);
    hopper.setduty(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}