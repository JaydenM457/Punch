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
    //     return m_shooter.setVelocity(RPM.of(5000)).withTimeout(1)
    //     .alongWith(m_indexer.set(-1).withTimeout(1)) 
    //     .andThen(
    //         m_intake.set(-1) 
    //          .alongWith(m_shooter.setVelocity(RPM.of(5000)))
    //         .alongWith(m_indexer.set(-1)) 
    //         .alongWith(m_Hopper.set(-1)))
        
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
                Intake.set(THROWUP_COMMAND_CONSTANTS.INTAKE_OUT_HALF)
            .alongWith(Indexer.set(THROWUP_COMMAND_CONSTANTS.INDEXER_OUT_HALF)
            .alongWith(Shooter.setVelocity(THROWUP_COMMAND_CONSTANTS.SHOOTER_OUT)) 
            .alongWith(Hopper.set(THROWUP_COMMAND_CONSTANTS.HOPPER_OUT_HALF))))
            .beforeStarting(() -> SmartDashboard.putBoolean("Throwup", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Throwup", false));
    
    }


    public Command armOscillate() {
        return Arm.setAngleAndStop(Degrees.of(150))
            .andThen(Arm.setAngleAndStop(Degrees.of(100)))
            .repeatedly()
                .beforeStarting(() -> SmartDashboard.putBoolean("Arm Oscillating", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Arm Oscillating", false));
    }
}
