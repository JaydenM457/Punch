// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.CommandTrain;
import frc.robot.commands.ShootCommand;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HopperSubsytem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Degrees;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  

  private final  CommandPS5Controller driverController = new CommandPS5Controller(0);
  private final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);

   private final ArmSubsystem m_arm = new ArmSubsystem();

  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
   private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final HopperSubsytem m_Hopper = new HopperSubsytem();

  // Systems (command factories)
  private final CommandTrain m_fuelSystem = new CommandTrain(
          m_arm,
           m_indexer, 
          m_intake,
           m_shooter, m_Hopper
  );

    private final AutoCommands a_Commands = new AutoCommands(
          m_arm,
           m_indexer, 
          m_intake,
           m_shooter, m_Hopper
  );


  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));

  // /**
  //  * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  //  */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverController.getLeftY() * 1,
                                                                () -> -driverController.getLeftX() * 1)
                                                                .withControllerRotationAxis(()->-driverController.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);


  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverController.getLeftY(),
                                                                () -> -driverController.getLeftX())
                                                                .withControllerRotationAxis(() -> driverController.getRawAxis(
                                                                2))
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                .withControllerHeadingAxis(() ->
                                                                Math.sin(
                                                                driverController.getRawAxis(
                                                                2) *
                                                                Math.PI) *
                                                                (Math.PI *
                                                                2),
                                                                () ->
                                                                Math.cos(
                                                                driverController.getRawAxis(
                                                                2) *
                                                                Math.PI) *
                                                                (Math.PI *
                                                                2))
                                                               .headingWhile(true);
  SendableChooser<Command> autChooser;


  public RobotContainer()
  {


    m_indexer.setDefaultCommand(m_indexer.set(0));
     m_intake.setDefaultCommand(m_intake.set(0));
    m_shooter.setDefaultCommand(m_shooter.set(0));
    
    m_Hopper.setDefaultCommand(m_Hopper.set(0));
    //m_arm.setDefaultCommand(m_arm.setAngle(Degrees.of(-40)));

  




    CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    //NamedCommands.registerCommand("TimedShoot", m_fuelSystem.timedShoot());
    //NamedCommands.registerCommand("TimedIntaking", m_fuelSystem.timedIntaking());
    NamedCommands.registerCommand("Shoot", new AutoShoot(()->RPM.of(3000), 
                                      m_shooter, m_indexer, m_Hopper, 3));
    NamedCommands.registerCommand("Intake", a_Commands.Auto_Intaking());
    NamedCommands.registerCommand("ArmDown", m_arm.setAngleAndStop(Degrees.of(230)));

    autChooser = AutoBuilder.buildAutoChooser("MiddleAuto");
    SmartDashboard.putData("Auto Chooser",autChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandPS5Controller Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // new Trigger(() -> 
    //     m_operatorController.L1().getAsBoolean() || 
    //     m_operatorController.L2().getAsBoolean() ||
    //     m_operatorController.R1().getAsBoolean() ||
    //     m_operatorController.R2().getAsBoolean() 
    // ).whileTrue(
    //     m_arm.setAngle(Degrees.of(180))
    // ).onFalse(
    //     m_arm.setAngle(Degrees.of(235))
    // );

    
    // m_operatorController.R2().whileTrue(new ShootCommand(() -> RPM.of(3100),  
    // m_shooter, m_indexer, m_Hopper));
    // m_operatorController.R1().whileTrue(new ShootCommand(() -> RPM.of(2900),  
    // m_shooter, m_indexer, m_Hopper));
    // m_operatorController.L1().whileTrue(new ShootCommand(() -> RPM.of(2500),  
    // m_shooter, m_indexer, m_Hopper));
    // m_operatorController.L2().whileTrue(new ShootCommand(() -> RPM.of(2200),  
    // m_shooter, m_indexer, m_Hopper));

    // //m_operatorController.button(3).whileTrue(m_fuelSystem.armOscillate());
    // m_operatorController.triangle().whileTrue(m_fuelSystem.Intaking());
    // //m_operatorController.R2().whileTrue(m_fuelSystem.shoot());
    // m_operatorController.square().whileTrue(m_fuelSystem.throwup());
    // m_operatorController.cross().onTrue(m_fuelSystem.mixer());
    // //m_operatorController.L1().onFalse(m_arm.setAngle(Degrees.of(235)));
    // driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    m_operatorController.button(3).onTrue(new AutoShoot(()->RPM.of(3000), 
                                      m_shooter, m_indexer, m_Hopper, 7));
    
    //m_operatorController.cross().onTrue(m_fuelSystem.mixer());
    //m_operatorController.square().onTrue(m_arm.setAngle(Degrees.of(90)));
    //m_operatorController.triangle().onTrue(m_arm.setAngle(Degrees.of(235)));
    // m_operatorController.button(1).whileTrue(m_fuelSystem.Intaking());
    // m_operatorController.button(1).onFalse(m_arm.setAngle(Degrees.of(150)));
    // m_operatorController.button(2).whileTrue(m_fuelSystem.shoot());
    // m_operatorController.button(3).whileTrue(m_fuelSystem.throwup());
    // m_operatorController.button(4).onTrue(m_fuelSystem.mixer());

    //  m_operatorController.R1().whileTrue(m_arm.sysId());
    //  m_operatorController.L2().whileTrue(m_arm.setAngle(Degrees.of(230)));
    //  m_operatorController.R2().whileTrue(m_arm.setAngle(Degrees.of(90)));
    // m_operatorController.rightTrigger().whileTrue(m_shooter.set(1));
    // m_operatorController.button(4).onTrue(m_arm.setAngle(Degrees.of(60)));
    // m_operatorController.button(5).onTrue(m_arm.setAngle(Degrees.of(120)));
    // m_operatorController.button(6).onTrue(m_arm.setAngle(Degrees.of(200)));
    // m_operatorController.button(7).onTrue(m_arm.setAngle(Degrees.of(-1)));
    // m_operatorController.button(1).whileTrue(m_shooter.set(0));
    

    // // m_operatorController.button(2).onTrue(m_arm.setAngle(Degrees.of(-100)));
    //m_operatorController.button(1).whileTrue(m_fuelSystem.armOscillate());
    // //m_operatorController.button(1).whileTrue(m_fuelSystem.throwup());
    // // m_operatorController.button(3).onFalse(m_arm.setAngle(Degrees.of(200)));
    //m_operatorController.button(1).whileTrue(new ShootCommand(()-> RPM.of(5000), m_shooter, m_indexer, m_Hopper));




    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // /**
    //  * Here we declare all of our operator commands, these commands could have been
    //  * written more compact but are left verbose so the intent is clear.
    //  */

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {                                                                                                                                 
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverController.create().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverController.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.triangle().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverController.create().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.options().whileTrue(drivebase.centerModulesCommand());
      //driverController.L1().onTrue(Commands.none());
      driverController.R1().onTrue(Commands.none());
    } else
    {
      //driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverController.circle().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverController.create().whileTrue(Commands.none());
      driverController.options().whileTrue(Commands.none());
      //driverController.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.R1().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autChooser.getSelected();//drivebase.getAutonomousCommand("RightAuto");
  }

  public void setMotorBrake(boolean brake)
  {
   drivebase.setMotorBrake(brake);
  }
}
