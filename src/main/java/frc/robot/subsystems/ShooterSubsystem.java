package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {
  private SparkFlex shooterMotor = new SparkFlex(SHOOTER_CONSTANTS.SHOOTER_ID, MotorType.kBrushless);
  private SparkFlex shootertwo = new SparkFlex(SHOOTER_CONSTANTS.SHOOTER_TWO_ID, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(0.0128, 0, 0)
  .withSimClosedLoopController(0, 0, 0)
  .withFeedforward(new SimpleMotorFeedforward(0.15, 0.1011, 0))
  //.withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40))//////
  .withFollowers(Pair.of(shootertwo, true));

  private SmartMotorController sparkSmartMotorController = new SparkWrapper(shooterMotor, DCMotor.getNeoVortex(2), smcConfig);

 private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
  .withDiameter(Inches.of(2))
  .withMass(Pounds.of(1))
  .withUpperSoftLimit(RPM.of(6500))
  .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel(shooterConfig);

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
    }   

    public void setMechanismVelocitySetpoint(AngularVelocity speed){
      shooter.setMechanismVelocitySetpoint(speed);
    }
  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed);
    }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return shooter.set(dutyCycle);
    }

        public void setduty(double dutyCycle) {
    shooter.set(dutyCycle);
}

  public ShooterSubsystem() {}

  

  @Override
  public void periodic() {

    shooter.updateTelemetry();

    double rpm = shooter.getSpeed().in(RPM);
    SmartDashboard.putNumber("Shooter RPM", rpm);
  }

  @Override
  public void simulationPeriodic() {
  
    shooter.simIterate();
  }


}


