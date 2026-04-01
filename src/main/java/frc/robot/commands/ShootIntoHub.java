package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsytem;

public class ShootIntoHub extends Command
{
    private SwerveSubsystem swerveDrive;
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private Cameras camera;
    private Command armOscillateCommand;
    //private Command mixer;
    private HopperSubsytem Hopper;
    private AngularVelocity rotationsSetpoint;
    private int[] hubAprilTagIDs;
    private int closestHubAprilTagID = -1;

    /*
     * Interpolating Double Tree Map
     * For each entry, the first value (k) represents the distance in meters from
     * the hub and the second value (v) represents the RPM needed
     * to accurately shoot into the hub at that distance.
     * Format: Map.entry(distanceInMeters, RPM)
     */
    private InterpolatingDoubleTreeMap interpolatingMap = new InterpolatingDoubleTreeMap().ofEntries(
        Map.entry(1.8, 2600.0),
        Map.entry(1.9, 2700.0),
        Map.entry(2.0, 2800.0),
        Map.entry(2.1, 2900.0),
        Map.entry(2.2, 3000.0),
        Map.entry(2.5, 3300.0),
        Map.entry(2.8, 3600.0),
        Map.entry(2.9, 3700.0),
        Map.entry(3.0, 3800.0),
        Map.entry(3.1, 3900.0));
    

    public ShootIntoHub(Cameras camera, int[] hubAprilTagIDs, ShooterSubsystem shooter, IndexerSubsystem indexer, HopperSubsytem Hopper, SwerveSubsystem swerveSubsystem, Command armOscillate)
    {
      this.camera = camera;
      this.shooter = shooter;
      this.indexer = indexer;
      this.Hopper = Hopper;
      this.hubAprilTagIDs = hubAprilTagIDs;
      this.armOscillateCommand = armOscillate;
      this.swerveDrive = swerveSubsystem;
      //this.mixer = mixer;
      addRequirements(shooter, indexer, Hopper);
    }
    
    
  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    //CommandScheduler.getInstance().schedule(mixer);
    
    
    
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    if (closestHubAprilTagID == -1) { // If the closest AprilTag hasn't been found yet
      Optional<PhotonPipelineResult> cameraInitialResult = camera.getBestResult();

      if (cameraInitialResult.isPresent()) {
        PhotonPipelineResult cameraResult = cameraInitialResult.get();
        ArrayList<PhotonTrackedTarget> closestTargets = camera.getClosestTargets(cameraResult, swerveDrive.getVision());

        if (closestTargets == null) {
            return;
        }

        for (PhotonTrackedTarget target: closestTargets) {
          if (target != null) {
            for (int aprilTagID: hubAprilTagIDs) {
              if (target.getFiducialId() == aprilTagID) {
                closestHubAprilTagID = target.getFiducialId();

                return;
              }
            }
          }
        }
      }
    }
    else { // Once the closest hub AprilTag has been found, then only calculate the distance to that tag
      double distanceToHubAprilTag = Cameras.LEFT_CAM.getDistanceToHub(swerveDrive.getVision(), new int[]{Constants.blueZoneHubLeftTagID,
          Constants.blueZoneHubRightTagID,
          Constants.redZoneHubLeftTagID,
          Constants.redZoneHubRightTagID,
          Constants.blueZoneHubCenterTagID,
          Constants.redZoneHubCenterTagID,
          Constants.blueZoneHubCenterLeftTagID,
          Constants.redZoneHubCenterLeftTagID
        }, false);

      rotationsSetpoint = RPM.of(interpolatingMap.get(distanceToHubAprilTag));

      shooter.setMechanismVelocitySetpoint(rotationsSetpoint);

      if (shooter.getVelocity().in(RPM) >= rotationsSetpoint.in(RPM) * 0.95) {
        //CommandScheduler.getInstance().cancel(mixer);
        indexer.setduty(-1);
        Hopper.setduty(-1);
        CommandScheduler.getInstance().schedule(armOscillateCommand);
      }
      else {
        indexer.setduty(0);
        Hopper.setduty(0);
      }
      
      return;
    }
    
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    closestHubAprilTagID = -1;
    CommandScheduler.getInstance().cancel(armOscillateCommand);
    shooter.setduty(0);
  }
}
