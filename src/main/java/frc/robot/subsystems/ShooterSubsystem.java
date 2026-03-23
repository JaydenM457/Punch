package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  .withClosedLoopController(SHOOTER_CONSTANTS.kP, SHOOTER_CONSTANTS.kI, SHOOTER_CONSTANTS.kD)
  //.withSimClosedLoopController(0, 0, 0)
  .withFeedforward(new SimpleMotorFeedforward(SHOOTER_CONSTANTS.kS, SHOOTER_CONSTANTS.kV, SHOOTER_CONSTANTS.kA))
  //.withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,1)))
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40))
  .withFollowers(Pair.of(shootertwo, true));

  private SmartMotorController sparkSmartMotorController = new SparkWrapper(shooterMotor, DCMotor.getNeoVortex(2), smcConfig);

 private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
  .withDiameter(Inches.of(2))
  .withMass(Pounds.of(1))
  .withUpperSoftLimit(SHOOTER_CONSTANTS.LOWER_SOFT_LIMIT)
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
  public double rpmForDistanceMeters(double distanceMeters) {
  // EXAMPLE ONLY ACTUALLY TEST WITH REAL DATA TO FIND VALUES
  if (distanceMeters <= 0.25) return 2000;
  if (distanceMeters <= 0.3) return 2050;
  if (distanceMeters <= 0.35) return 2075;
  if (distanceMeters <= 0.4) return 2100;
  if (distanceMeters <= 0.45) return 2150;
  if (distanceMeters <= 0.475) return 2175;
  if (distanceMeters <= 0.5) return 2200;
  if (distanceMeters <= 0.55) return 2250;
  if (distanceMeters <= 0.6) return 2300;
  if (distanceMeters <= 0.65) return 2350;
  if (distanceMeters <= 0.7) return 2400;
  if (distanceMeters <= 0.75) return 2450;
  if (distanceMeters <= 0.8) return 2500;
  if (distanceMeters <= 0.85) return 2550;
  if (distanceMeters <= 0.9) return 2600;
  if (distanceMeters <= 0.95) return 2650;
  if (distanceMeters <= 1.0) return 2700;
  if (distanceMeters <= 1.25) return 2750;
  if (distanceMeters <= 1.5) return 2800;
  if (distanceMeters <= 1.75) return 2850;
  if (distanceMeters <= 1.8) return 2600;
  if (distanceMeters <= 1.9) return 2700;
  if (distanceMeters <= 2.0) return 2800;
  if (distanceMeters <= 2.1) return 2900;
  if (distanceMeters <= 2.2) return 3000;
  if (distanceMeters <= 2.3) return 3100;
  if (distanceMeters <= 2.4) return 3200;
  if (distanceMeters <= 2.5) return 3300;
  if (distanceMeters <= 2.6) return 3350;
  if (distanceMeters <= 2.7) return 3400;
  if (distanceMeters <= 2.8) return 3600;
  if (distanceMeters <= 2.9) return 3700;
  if (distanceMeters <= 3.0) return 3800;
  if (distanceMeters <= 3.1) return 3900;
  if (distanceMeters <= 3.5) return 4000;
  if (distanceMeters <= 4.0) return 4300;
  if (distanceMeters <= 4.5) return 4600;
  return 4600 + (distanceMeters - 4.5) * 600; 

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


