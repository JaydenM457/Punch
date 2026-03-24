package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.util.ShotSolver;

public class ShootOnTheMove extends Command {

  private final SwerveSubsystem swerve;
  private final Vision vision;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final HopperSubsytem hopper;

  private final DoubleSupplier xInput;
  private final DoubleSupplier yInput;
  private final BooleanSupplier feedButton;
  private Command armOscillateCommand;

  private final PIDController thetaPID =
      new PIDController(
          Constants.HUB_THETA_KP,
          Constants.HUB_THETA_KI,
          Constants.HUB_THETA_KD);

  private int lockedTag = -1;
  private Translation2d hubCenter = null;
  private boolean seeingTag = false;

  public ShootOnTheMove(
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

    thetaPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, shooter, indexer, hopper);
  }

  @Override
  public void initialize() {
    lockedTag = -1;
    seeingTag = false;
    thetaPID.reset();

    hubCenter =
        vision.getHubCenterFieldPositionFromLayout(Constants.HUB_TAG_IDS).orElse(null);
  }

  @Override
  public void execute() {

    Optional<Integer> visibleTag = vision.getBestVisibleHubTag(Constants.HUB_TAG_IDS);
    seeingTag = visibleTag.isPresent();

    if (visibleTag.isPresent()) {
      lockedTag = visibleTag.get();
    }

    if (lockedTag != -1) {
      vision.getHubCenterFieldPositionFromTag(lockedTag)
          .ifPresent(t -> hubCenter = t);
    }

    Pose2d pose = swerve.getPose();

    // Driver translation
    ChassisSpeeds commanded =
        swerve.getTargetSpeeds(
            xInput.getAsDouble(),
            yInput.getAsDouble(),
            pose.getRotation());

    // Measured velocity
    ChassisSpeeds measured = swerve.getFieldVelocity();

    Translation2d commandedVel =
        new Translation2d(commanded.vxMetersPerSecond, commanded.vyMetersPerSecond);

    Translation2d measuredVel =
        new Translation2d(measured.vxMetersPerSecond, measured.vyMetersPerSecond);

    Translation2d estimatedVel =
        measuredVel.times(Constants.MEASURED_VELOCITY_WEIGHT)
            .plus(commandedVel.times(1.0 - Constants.MEASURED_VELOCITY_WEIGHT));

    if (hubCenter == null) {
      swerve.driveFieldOriented(commanded);
      return;
    }

    Optional<ShotSolver.ShotSolution> solutionOpt =
        ShotSolver.solveMovingShot(
            pose.getTranslation(),
            estimatedVel,
            hubCenter);

    if (solutionOpt.isEmpty()) {
      swerve.driveFieldOriented(commanded);
      return;
    }

    var sol = solutionOpt.get();

    shooter.setMechanismVelocitySetpoint(RPM.of(sol.shooterRPM()));

    double omega =
        thetaPID.calculate(
            pose.getRotation().getRadians(),
            sol.correctedHeading().getRadians());

    omega = MathUtil.clamp(omega,
        -Constants.HUB_MAX_OMEGA_RAD_PER_SEC,
        Constants.HUB_MAX_OMEGA_RAD_PER_SEC);

    swerve.driveFieldOriented(
        new ChassisSpeeds(
            commanded.vxMetersPerSecond,
            commanded.vyMetersPerSecond,
            omega));

    double headingError =
        Math.abs(pose.getRotation().minus(sol.correctedHeading()).getDegrees());
    if (headingError > 180) headingError = 360 - headingError;

    double tolerance =
        seeingTag
            ? Constants.HUB_HEADING_TOLERANCE_DEG_WITH_TAG
            : Constants.HUB_HEADING_TOLERANCE_DEG_NO_TAG;

    boolean drivetrainReady = headingError <= tolerance;
    boolean shooterReady =
        shooter.getVelocity().in(RPM)
            >= sol.shooterRPM() * Constants.SHOOTER_READY_FRACTION;

    boolean shouldFeed =
        feedButton.getAsBoolean() && drivetrainReady && shooterReady;

    if (shouldFeed) {
      indexer.setduty(-1);
      hopper.setduty(-1);
      CommandScheduler.getInstance().schedule(armOscillateCommand);
    } else {
      indexer.setduty(0);
      hopper.setduty(0);
    }

    SmartDashboard.putNumber("SOTM/RPM", sol.shooterRPM());
    SmartDashboard.putNumber("SOTM/TOF", sol.timeOfFlightSeconds());
    SmartDashboard.putNumber("SOTM/HeadingError", headingError);
    SmartDashboard.putBoolean("SOTM/Feeding", shouldFeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.set(0);
    indexer.setduty(0);
    hopper.setduty(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}