
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER_CONSTANTS;

public class IndexerSubsystem extends SubsystemBase {
    private SparkFlex indexerMotor = new SparkFlex(INDEXER_CONSTANTS.INDEXER_ID, MotorType.kBrushless);




  /** Creates a new ExampleSubsystem. */
  public IndexerSubsystem() {
    //setDefaultCommand(indexerMotor.set(0));



    SparkFlexConfig indexerConfig = new SparkFlexConfig();
    indexerConfig.smartCurrentLimit(60);
    indexerConfig.idleMode(IdleMode.kCoast);
    indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    public Command set(double dutycycle)
  {
    return run(()->indexerMotor.set(dutycycle));
  }

    public void setduty(double dutyCycle) {
    indexerMotor.set(dutyCycle);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }



  @Override
  public void simulationPeriodic() {

  }
}


