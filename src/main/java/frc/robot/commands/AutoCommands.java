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
import frc.robot.Constants.COMMAND_TRAIN_CONSTANTS.SHOOTER_SPEED;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HopperSubsytem;


public class AutoCommands {
    
     private ArmSubsystem Arm;
    private IndexerSubsystem Indexer;
    private IntakeSubsystem Intake;
    private ShooterSubsystem Shooter;
    private HopperSubsytem Hopper;
    

    public AutoCommands(ArmSubsystem Arm,
     IndexerSubsystem Indexer, 
        IntakeSubsystem Intake, 
        ShooterSubsystem Shooter, 
        HopperSubsytem Hopper){
        this.Arm = Arm;
        this.Indexer = Indexer;
        this.Intake = Intake;
        this.Shooter = Shooter;
        this.Hopper = Hopper;

    }


    public Command Auto_Intaking(){

        return Arm.setAngleAndStop(COMMAND_TRAIN_CONSTANTS.DOWN_ANGLE)
        .andThen(
            Intake.set(INTAKING_COMMAND_CONSTANTS.INTAKE_INTAKE_SPEED)
        .alongWith(Hopper.set(INTAKING_COMMAND_CONSTANTS.HOPPER_INTAKE_SPEED)))
        .beforeStarting(() -> SmartDashboard.putBoolean("AutoIntaking", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("AutoIntaking", false));
    }

        public Command Auto_STOP(){

        return Intake.set(0).withTimeout(0.00001)
        .alongWith(Hopper.set(0)).withTimeout(0.0001)
        .alongWith(Shooter.set(0)).withTimeout(0.00001)
        .alongWith(Indexer.set(0)).withTimeout(0.00001);

    }
    
    // public Command shoot(){ 
    //     return Shooter.setVelocity(SHOOTER_SPEED.SIDE_TRENCH_VELOCITY).withTimeout(1.5)
    //     // .alongWith(Indexer.set(-1).withTimeout(1)) 
    //     .andThen(Shooter.setVelocity(SHOOTER_SPEED.SIDE_TRENCH_VELOCITY)
    //         .alongWith(Indexer.set(-1)) 
    //         .alongWith(Hopper.set(-1))
    //         .alongWith(Intake.set(-0.8))
    //         .alongWith()).withTimeout(6);
    // }

    //     public Command shoot_Corrner(){ 
    //     return Shooter.setVelocity(SHOOTER_SPEED.CORRNER_VELOCITY).withTimeout(1.5)
    //     // .alongWith(Indexer.set(-1).withTimeout(1)) 
    //     .andThen(Shooter.setVelocity(SHOOTER_SPEED.CORRNER_VELOCITY)
    //         .alongWith(Indexer.set(-1)) 
    //         .alongWith(Hopper.set(-1))).withTimeout(6);
        

    // }
    //         public Command shoot_strait(){ 
    //     return Shooter.setVelocity(RPM.of(2300)).withTimeout(1.5)
    //     // .alongWith(Indexer.set(-1).withTimeout(1)) 
    //     .andThen(Shooter.setVelocity(RPM.of(2300))
    //         .alongWith(Indexer.set(-1)) 
    //         .alongWith(Hopper.set(-1))).withTimeout(6);
        

    // }
    


}