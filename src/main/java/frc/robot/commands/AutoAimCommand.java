package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldConstants.Hub;
import java.util.List;
import swervelib.SwerveInputStream;


public class AutoAimCommand extends Command
{

  private final SwerveSubsystem   swerveSubsystem;
  private final SwerveInputStream swerveInputStream;

  public AutoAimCommand(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy();
//    this.swerveInputStream.scaleTranslation(slowScale);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize()
  {
    Pose2d targetPose = AllianceFlipUtil.apply(new Pose2d(Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero));
    swerveSubsystem.getField().getObject("AimTarget").setPose(targetPose);

    swerveInputStream.aim(targetPose)
//                     .aimHeadingOffset(Rotation2d.kZero)
//                     .aimHeadingOffset(true)
                     .aimWhile(true);

  }

  @Override
  public void execute()
  {
    swerveSubsystem.driveFieldOrientedSetpoint(swerveInputStream.get());
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    swerveInputStream.aimWhile(false);
    swerveSubsystem.getField().getObject("AimTarget").setPoses(List.of());
  }
}
