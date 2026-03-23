package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.FerrySide;
import frc.robot.subsystems.HopperSubsytem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FerryShot extends Command {
  private final SwerveSubsystem swerve;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final HopperSubsytem hopper;
  private Command armOscillateCommand;

  private final DoubleSupplier xInput;
  private final DoubleSupplier yInput;
  private final Supplier<FerrySide> ferrySideSupplier;

  // If true, feed automatically when ready.
  // If false, just aim + spin up.
  private final BooleanSupplier autoFeedSupplier;

  public FerryShot(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      HopperSubsytem hopper,
      DoubleSupplier xInput,
      DoubleSupplier yInput,
      Supplier<FerrySide> ferrySideSupplier,
      BooleanSupplier autoFeedSupplier) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.indexer = indexer;
    this.hopper = hopper;
    this.xInput = xInput;
    this.yInput = yInput;
    this.ferrySideSupplier = ferrySideSupplier;
    this.autoFeedSupplier = autoFeedSupplier;

    addRequirements(swerve, shooter, indexer, hopper);
  }

  @Override
  public void execute() {
    Pose2d robotPose = swerve.getPose();

    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    FerrySide side = ferrySideSupplier.get();

    Rotation2d targetHeading = getTargetHeading(side, isRedAlliance);

    ChassisSpeeds driveSpeeds =
        swerve.getTargetSpeeds(
            xInput.getAsDouble(),
            yInput.getAsDouble(),
            targetHeading);

    swerve.driveFieldOriented(driveSpeeds);

    double distanceFromDriverStation =
        Constants.getDistanceFromDriverStationWall(robotPose.getX(), isRedAlliance);

    double shooterRPM = Constants.getFerryRPM(distanceFromDriverStation);
    shooter.setMechanismVelocitySetpoint(RPM.of(shooterRPM));

    double headingErrorDeg =
        Math.abs(robotPose.getRotation().minus(targetHeading).getDegrees());
    if (headingErrorDeg > 180.0) {
      headingErrorDeg = 360.0 - headingErrorDeg;
    }

    boolean drivetrainReady = headingErrorDeg <= Constants.FERRY_HEADING_TOLERANCE_DEG;
    boolean shooterReady =
        shooter.getVelocity().in(RPM) >= shooterRPM * Constants.FERRY_SHOOTER_READY_FRACTION;

    boolean shouldFeed = autoFeedSupplier.getAsBoolean() && drivetrainReady && shooterReady;

    if (shouldFeed) {
      indexer.setduty(-1.0);
      hopper.setduty(-1.0);
      CommandScheduler.getInstance().schedule(armOscillateCommand);
    } else {
      indexer.setduty(0.0);
      hopper.setduty(0.0);
    }

    SmartDashboard.putString("Ferry/Side", side.name());
    SmartDashboard.putBoolean("Ferry/IsRedAlliance", isRedAlliance);
    SmartDashboard.putNumber("Ferry/RobotX", robotPose.getX());
    SmartDashboard.putNumber("Ferry/DistanceFromDriverStation", distanceFromDriverStation);
    SmartDashboard.putNumber("Ferry/TargetHeadingDeg", targetHeading.getDegrees());
    SmartDashboard.putNumber("Ferry/HeadingErrorDeg", headingErrorDeg);
    SmartDashboard.putNumber("Ferry/ShooterTargetRPM", shooterRPM);
    SmartDashboard.putBoolean("Ferry/DrivetrainReady", drivetrainReady);
    SmartDashboard.putBoolean("Ferry/ShooterReady", shooterReady);
    SmartDashboard.putBoolean("Ferry/Feeding", shouldFeed);
  }

  private Rotation2d getTargetHeading(FerrySide side, boolean isRedAlliance) {
    if (isRedAlliance) {
      return side == FerrySide.LEFT_BACK
          ? Constants.FERRY_LEFT_CORNER_HEADING_RED
          : Constants.FERRY_RIGHT_CORNER_HEADING_RED;
    } else {
      return side == FerrySide.LEFT_BACK
          ? Constants.FERRY_LEFT_CORNER_HEADING_BLUE
          : Constants.FERRY_RIGHT_CORNER_HEADING_BLUE;
    }
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