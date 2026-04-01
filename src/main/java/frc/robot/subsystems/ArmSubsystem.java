package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM_CONSTANTS;
import frc.robot.Constants.ARM_CONSTANTS.ARM_FOLLOWER_CONSTANTS;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase
{
  
  private final Mass     weight = Pounds.of(7);
  private final DCMotor  motors = DCMotor.getNEO(2);
  private final Distance length = Inches.of(14);
  private final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(75));


  private final SparkMax LeaderArmMotor  = new SparkMax(ARM_CONSTANTS.ARM_ID, MotorType.kBrushless);
  private final SparkMax FollowerArmMotor  = new SparkMax(ARM_FOLLOWER_CONSTANTS.ARM_TWO_ID, MotorType.kBrushless);


  private SparkAbsoluteEncoder leaderAbsoluteEncoder = LeaderArmMotor.getAbsoluteEncoder();
  private SparkAbsoluteEncoder followerAbsoluteEncoder = FollowerArmMotor.getAbsoluteEncoder();



  private final SmartMotorControllerConfig followerMotorConfig = new SmartMotorControllerConfig(this)
       .withClosedLoopController(ARM_FOLLOWER_CONSTANTS.kP, ARM_FOLLOWER_CONSTANTS.kI, ARM_FOLLOWER_CONSTANTS.kD)
      .withSoftLimit(Degrees.of(0), Degrees.of(130.0))

      // .withClosedLoopController(new ExponentialProfilePIDController(ARM_FOLLOWER_CONSTANTS.kP, ARM_FOLLOWER_CONSTANTS.kI, ARM_FOLLOWER_CONSTANTS.kD, ExponentialProfilePIDController
      // .createArmConstraints(Volts.of(10), motors, weight, length, gearing)))
      // .withSoftLimit(Rotations.of(0.501), Rotations.of(0.883))
      .withGearing(gearing)
      .withExternalEncoder(followerAbsoluteEncoder)
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ArmFollowerMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(20))
      .withMotorInverted(false)
      // .withClosedLoopRampRate(Seconds.of(0.25))
      // .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(ARM_FOLLOWER_CONSTANTS.kS, ARM_FOLLOWER_CONSTANTS.kG, ARM_FOLLOWER_CONSTANTS.kV, ARM_FOLLOWER_CONSTANTS.kA))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withExternalEncoderInverted(false)
      
      .withExternalEncoderZeroOffset(Degrees.of(179.3))
      .withUseExternalFeedbackEncoder(true)
      .withVendorConfig(new SparkMaxConfig().apply(new AbsoluteEncoderConfig().zeroCentered(true)));

      ;


  private final SmartMotorController       followerMotorController            = new SparkWrapper(FollowerArmMotor,
                                                                               DCMotor.getNEO(2),
                                                                               followerMotorConfig);





  private final SmartMotorControllerConfig leaderMotorConfig = new SmartMotorControllerConfig(this)
       .withClosedLoopController(ARM_CONSTANTS.kP, ARM_CONSTANTS.kI, ARM_CONSTANTS.kD)

      // .withClosedLoopController(new ExponentialProfilePIDController(ARM_CONSTANTS.kP, ARM_CONSTANTS.kI, ARM_CONSTANTS.kD, ExponentialProfilePIDController
      // .createArmConstraints(Volts.of(10), motors, weight, length, gearing)))
      .withSoftLimit(Degrees.of(0), Degrees.of(130.0))
      .withGearing(gearing)
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ArmLeaderMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(20))
      .withMotorInverted(true)
      // .withClosedLoopRampRate(Seconds.of(0.25))
      // .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(ARM_CONSTANTS.kS, ARM_CONSTANTS.kG, ARM_CONSTANTS.kV, ARM_CONSTANTS.kA))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withLooselyCoupledFollowers(followerMotorController)
      .withExternalEncoder(leaderAbsoluteEncoder)
      .withExternalEncoderInverted(true)
      .withExternalEncoderZeroOffset(Degrees.of(-30.12+360.0)) // Remove if configured in REV HW Client
      .withUseExternalFeedbackEncoder(true)
      .withVendorConfig(new SparkMaxConfig().apply(new AbsoluteEncoderConfig().zeroCentered(true)));




  private final SmartMotorController       leaderMotorController            = new SparkWrapper(LeaderArmMotor,
                                                                               DCMotor.getNEO(2),
                                                                               leaderMotorConfig);





  private ArmConfig m_config = new ArmConfig(leaderMotorController)
      .withLength(length)
      .withHardLimit(Rotations.of(-12), Rotations.of(140))
      .withTelemetry("ArmSubsystem", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(7))
      .withSimStartingPosition(Rotations.of(0));
      //.withStartingPosition(Degrees.of(0));




  private final Arm  arm  = new Arm(m_config);

  public ArmSubsystem()
  {
  }

  public void periodic()
  {
     arm.updateTelemetry();
     followerMotorController.updateTelemetry();
    //     SmartDashboard.putNumber(
    //     "Arm Angle (Degrees)",
    //     arm.getAngle().in(Degrees)
    // );
    //     SmartDashboard.putNumber(
    //     "Arm Angle (Rotations)",
    //     arm.getAngle().in(Rotations)
    // );

  
  }

  public void simulationPeriodic()
  {
    arm.simIterate();
  }

  public Command armCmd(double dutycycle)
  {
    return arm.set(dutycycle);
  }

  public Command sysId()
  {
    return arm.sysId(Volts.of(1), Volts.of(1).per(Second), Second.of(30));
  }

  
  public Angle getAngle() {
    return arm.getAngle();
  }

  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }

  public Command setAngleAndStop(Angle angle)
  {
    return arm.runTo(angle, ARM_CONSTANTS.TOLERANCE);
  }
  



}
