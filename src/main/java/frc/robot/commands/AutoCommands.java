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


public class AutoCommands {
    
     private ArmSubsystem Arm;
    private IndexerSubsystem Indexer;
    private IntakeSubsystem Intake;
    private ShooterSubsystem Shooter;
    private HopperSubsytem Hopper;

    public AutoCommands(ArmSubsystem Arm, IndexerSubsystem Indexer, 
        IntakeSubsystem Intake, ShooterSubsystem Shooter, HopperSubsytem Hopper){
        this.Arm = Arm;
        this.Indexer = Indexer;
        this.Intake = Intake;
        this.Shooter = Shooter;
        this.Hopper = Hopper;

    }



    public Command Auto_Intaking(){

        return Arm.setAngleAndStop(Degrees.of(225))
        .andThen(Intake.set(-1)
        .alongWith(Hopper.set(-0.1)))
        .beforeStarting(() -> SmartDashboard.putBoolean("AutoIntaking", true))
        .finallyDo(interrupted -> SmartDashboard.putBoolean("AutoIntaking", false));
    }
    


}