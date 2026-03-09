package frc.robot.commands;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HopperSubsytem;


public class CommandTrain {
    
     private ArmSubsystem Arm;
    private IndexerSubsystem Indexer;
    private IntakeSubsystem Intake;
    private ShooterSubsystem Shooter;
    private HopperSubsytem Hopper;

    public CommandTrain(ArmSubsystem Arm, IndexerSubsystem Indexer, 
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

        return Arm.setAngle(Degrees.of(235))
        .alongWith(Intake.set(-1)
        .alongWith(Hopper.set(-0.1)))
        .beforeStarting(() -> SmartDashboard.putBoolean("Intaking", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Intaking", false));
    }

    public Command mixer(){
        return Hopper.set(1).withTimeout(0.2)
        .andThen(Hopper.set(-1).withTimeout(0.2))
        .andThen(Hopper.set(1).withTimeout(0.2))
        .beforeStarting(() -> SmartDashboard.putBoolean("Mixing", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Mixing", false));
    }
    

    public Command throwup(){
        return Arm.setAngle(Degrees.of(225))
            .alongWith(Intake.set(0.5)
            .alongWith(Indexer.set(0.5)
            .alongWith(Shooter.setVelocity(RPM.of(-500)) 
            .alongWith(Hopper.set(0.5)))))
            .beforeStarting(() -> SmartDashboard.putBoolean("Throwup", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("Throwup", false));
    
    }


    public Command armOscillate() {
        return Arm.setAngleAndStop(Degrees.of(180))
            .andThen(Arm.setAngleAndStop(Degrees.of(225)))
            .repeatedly();

    }

}