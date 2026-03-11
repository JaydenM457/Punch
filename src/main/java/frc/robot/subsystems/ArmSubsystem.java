package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM_CONSTANTS;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase
{
  
  private final Mass     weight = Pounds.of(3);
  private final DCMotor  motors = DCMotor.getNEO(1);
  private final Distance length = Inches.of(14);
  private final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(75));
  private final SparkMax ArmMotor  = new SparkMax(ARM_CONSTANTS.ARM_ID, MotorType.kBrushless);


  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      // .withClosedLoopController(0.0001, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))

      .withClosedLoopController(new ExponentialProfilePIDController(ARM_CONSTANTS.kP, ARM_CONSTANTS.kI, ARM_CONSTANTS.kD, ExponentialProfilePIDController
      .createArmConstraints(Volts.of(10), motors, weight, length, gearing)))
      .withSoftLimit(Rotations.of(0.032), Rotations.of(0.414))
      .withGearing(gearing)
      .withExternalEncoder(ArmMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(ARM_CONSTANTS.kS, ARM_CONSTANTS.kG, ARM_CONSTANTS.kV, ARM_CONSTANTS.kA))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor            = new SparkWrapper(ArmMotor,
                                                                               DCMotor.getNEO(1),
                                                                               motorConfig);
  private final MechanismPositionConfig    robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(0.25), Meters.of(0), Meters.of(0.5)));


  private       ArmConfig m_config = new ArmConfig(motor)
      .withLength(length)
      .withHardLimit(Rotations.of(0.032), Rotations.of(0.414))
      .withTelemetry("ArmSubsystem", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(3))
      //.withStartingPosition(Degrees.of(0))
      .withHorizontalZero(Degrees.of(0.348))
      .withMechanismPositionConfig(robotToMechanism);
  private final Arm       arm      = new Arm(m_config);

  public ArmSubsystem()
  {
  }

  public void periodic()
  {
    arm.updateTelemetry();
        SmartDashboard.putNumber(
        "Arm Angle (Degrees)",
        arm.getAngle().in(Degrees)
    );
        SmartDashboard.putNumber(
        "Arm Angle (Rotations)",
        arm.getAngle().in(Rotations)
    );

  
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

  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }

  public Command setAngleAndStop(Angle angle)
  {
    return arm.runTo(angle, ARM_CONSTANTS.TOLERANCE);
  }
  


}
