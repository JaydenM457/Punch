package frc.robot.commands;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.COMMAND_TRAIN_CONSTANTS;
import frc.robot.Constants.COMMAND_TRAIN_CONSTANTS.INTAKING_COMMAND_CONSTANTS;
import frc.robot.Constants.COMMAND_TRAIN_CONSTANTS.MIXER_COMMAND_CONSTANTS;
import frc.robot.Constants.COMMAND_TRAIN_CONSTANTS.THROWUP_COMMAND_CONSTANTS;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HopperSubsytem;


public class CommandTrain {
    
    private ArmSubsystem Arm;
    private IndexerSubsystem Indexer;
    private IntakeSubsystem Intake;
    private ShooterSubsystem Shooter;
    private HopperSubsytem Hopper;

    public CommandTrain(ArmSubsystem Arm,
     IndexerSubsystem Indexer, 
        IntakeSubsystem Intake, ShooterSubsystem Shooter, HopperSubsytem Hopper){
        this.Arm = Arm;
        this.Indexer = Indexer;
        this.Intake = Intake;
        this.Shooter = Shooter;
        this.Hopper = Hopper;

    }
    // +shooter = out
    //-shooter = in
    //+hopper/indexer = in
     //-hopper/indexer = out

    // public Command shoot(){ 
    //     return Shooter.setVelocity(RPM.of(2200)).withTimeout(3)
    //     // .alongWith(Indexer.set(-1).withTimeout(1)) 
    //     .andThen(
    //         Intake.set(-1) 
    //          .alongWith(Shooter.setVelocity(RPM.of(2200))
    //         .alongWith(Indexer.set(-1)) 
    //         .alongWith(Hopper.set(-1))).withTimeout(3))
        
    //    .beforeStarting(() -> SmartDashboard.putBoolean("Shoot Running", true))
    //    .finallyDo(interrupted -> SmartDashboard.putBoolean("Shoot Running", false));
    // }


    public Command Intaking(){

        return Arm.setAngleAndStop(COMMAND_TRAIN_CONSTANTS.DOWN_ANGLE)
       .andThen(
            Intake.set(INTAKING_COMMAND_CONSTANTS.INTAKE_INTAKE_SPEED)
        .alongWith(Hopper.set(INTAKING_COMMAND_CONSTANTS.HOPPER_INTAKE_SPEED)))
        .beforeStarting(() -> SmartDashboard.putBoolean("Intaking", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Intaking", false));
    }

    public Command mixer(){
        return Hopper.set(MIXER_COMMAND_CONSTANTS.HOPPER_OUT).withTimeout(0.2)
        .andThen(Hopper.set(MIXER_COMMAND_CONSTANTS.HOPPER_IN).withTimeout(0.2))
        .andThen(Hopper.set(MIXER_COMMAND_CONSTANTS.HOPPER_OUT).withTimeout(0.2))
        .beforeStarting(() -> SmartDashboard.putBoolean("Mixing", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Mixing", false));
    }
    

    public Command throwup(){
        return Arm.setAngleAndStop(COMMAND_TRAIN_CONSTANTS.DOWN_ANGLE)
            .alongWith(
                Intake.set(THROWUP_COMMAND_CONSTANTS.INTAKE_OUT)
            .alongWith(Indexer.set(THROWUP_COMMAND_CONSTANTS.INDEXER_OUT)
            .alongWith(Shooter.setVelocity(THROWUP_COMMAND_CONSTANTS.SHOOTER_OUT)) 
            .alongWith(Hopper.set(THROWUP_COMMAND_CONSTANTS.HOPPER_OUT))))
            .beforeStarting(() -> SmartDashboard.putBoolean("Throwup", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Throwup", false));
    
    }


    public Command armOscillate() {
        return Arm.setAngleAndStop(COMMAND_TRAIN_CONSTANTS.DOWN_ANGLE)
            .andThen(Arm.setAngleAndStop(COMMAND_TRAIN_CONSTANTS.SHOOT_ANGLE))
            //.repeatedly()
                .beforeStarting(() -> SmartDashboard.putBoolean("Arm Oscillating", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Arm Oscillating", false));
    }
    
    public Command PULSE() {
        return new Command() {
            double speed = -1.0;
            boolean goingUp = true;

            @Override
            public void execute() {

                if (goingUp) {
                    speed += 0.05;
                    if (speed >= -0.5) goingUp = false;
    
                } else {
                    speed -= 0.05;
                    if (speed <= -1.0) goingUp = true;
                }

                Hopper.set(speed).schedule();
                SmartDashboard.putNumber("Pulse Speed", speed);
            }

            @Override
            public void end(boolean interrupted) {
                Hopper.set(0).schedule(); // stop motor
            }

            @Override
            public boolean isFinished() {
                return false; 
            }
        }
        ;//.withTimeout(6.0); // <--- command stops after 2 seconds
    }

}
