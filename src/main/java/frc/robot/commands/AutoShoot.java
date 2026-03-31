package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HopperSubsytem;
import edu.wpi.first.wpilibj.Timer;

public class AutoShoot extends Command
{

    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private IntakeSubsystem Intake;
    //private Command armOscillateCommand;
    //private Command mixer;
    private HopperSubsytem Hopper;
    private Supplier<AngularVelocity> setpoint;
    private double time;
    private Timer timer = new Timer();
    private boolean feeding = false;
    

    public AutoShoot(Supplier<AngularVelocity> shootSpeed, 
                     ShooterSubsystem shooter, 
                     IndexerSubsystem indexer, 
                     HopperSubsytem Hopper, 
                     IntakeSubsystem Intake, 
                     //Command armOscillate,
                     double time
    )
    {
        this.shooter = shooter;
        this.indexer = indexer;
        this.Hopper = Hopper;
        this.Intake = Intake;
        setpoint = shootSpeed;
        this.time = time;
        //this.armOscillateCommand = armOscillate;
        //this.mixer = mixer;
        addRequirements(shooter, indexer, Hopper, Intake);
    }
    
    
  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    shooter.setMechanismVelocitySetpoint(setpoint.get());
    //CommandScheduler.getInstance().schedule(mixer);
      timer.reset();
      timer.stop();
      feeding = false;
    
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
@Override
public void execute()
{
    //CommandScheduler.getInstance().cancel(shooter.getDefaultCommand());
    shooter.setMechanismVelocitySetpoint(setpoint.get());


    if (shooter.getVelocity().in(RPM) >= setpoint.get().in(RPM) * 0.95)
    {
        if (!feeding)
        {
            timer.start();   // start timing ONLY once we hit speed
            feeding = true;
        }
        //shooter.setMechanismVelocitySetpoint(setpoint.get());
        indexer.setduty(-1);
        Hopper.setduty(-1);
        Intake.setduty(-0.8);
        //CommandScheduler.getInstance().schedule(armOscillateCommand);
    }
    else
    {
        indexer.setduty(0);
        Hopper.setduty(0);
        Intake.setduty(0);
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
    return feeding && timer.get() >= time;
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
 
    //CommandScheduler.getInstance().cancel(armOscillateCommand);
    shooter.setduty(0);
    indexer.setduty(0);
    Hopper.setduty(0);
    
    }
}
