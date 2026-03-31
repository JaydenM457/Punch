// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(13.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    // Joystick Deadband
    public static final double DEADBAND        = 0.3;
    public static final double LEFT_Y_DEADBAND = 0.3;
    public static final double RIGHT_X_DEADBAND = 0.3;
    public static final double TURN_CONSTANT    = 6;
  }

    public static class INDEXER_CONSTANTS{
    public static final int INDEXER_ID = 2;//SF
  }

  public static class INTAKE_CONSTANTS{
    public static final int INTAKE_ID = 3;//SM
  }

  public static class HOPPER_CONSTANTS{
    public static final int HOPPER_ID = 4;//SM
  }


  public static class ARM_CONSTANTS{
    public static final int ARM_ID = 6;//SM
    

    public static double kP = 5;
    public static double kD = 0;
    public static double kI = 0;

    public static double kS = 0;
    public static double kG = 0;
    public static double kV = 0;
    public static double kA = 0; 

    public static Angle LOWER_SOFT_LIMIT = Rotations.of(0.457);//0.457
    public static Angle UPPER_SOFT_LIMIT = Rotations.of(0.850);//0.844

    public static Angle LOWER_HARD_LIMIT = Rotations.of(0.457);//0.850
    public static Angle UPPER_HARD_LIMIT = Rotations.of(0.850);

    public static Angle HORIZONTAL_ZERO = Degrees.of(0.844);
    // public final static Mass     WEIGHT = Pounds.of(3);
    // public final static DCMotor  MOTORS = DCMotor.getNEO(1);
    // public final static Distance LENGTH = Inches.of(14);
    // public final static MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(75));
    

    public static class ARM_FOLLOWER_CONSTANTS{
      public static final int ARM_TWO_ID = 5;

      public static double kP = 5  ;
      public static double kD = 0;
      public static double kI = 0;

      public static double kS = 0;
      public static double kG = 0;
      public static double kV = 0;
      public static double kA = 0; 

      public static Angle LOWER_SOFT_LIMIT = Rotations.of(0.457);//0.457
      public static Angle UPPER_SOFT_LIMIT = Rotations.of(0.850);//0.844

      public static Angle LOWER_HARD_LIMIT = Rotations.of(0.457);//0.850
      public static Angle UPPER_HARD_LIMIT = Rotations.of(0.850);

      public static Angle HORIZONTAL_ZERO = Degrees.of(0.844);
      // public final static Mass     WEIGHT = Pounds.of(3);
      // public final static DCMotor  MOTORS = DCMotor.getNEO(1);
      // public final static Distance LENGTH = Inches.of(14);
      // public final static MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(75));
      // public static Angle TOLERANCE = Degrees.of(5);
      

    }
    public static Angle TOLERANCE = Degrees.of(2.5);

  }





  public static class SHOOTER_CONSTANTS{
    public static final int SHOOTER_ID = 7;//SF
    public static final int SHOOTER_TWO_ID = 8;//SF

    public static double kP = 0.0128;
    public static double kD = 0;
    public static double kI = 0;

    public static double kS = 0.15;
    public static double kV = 0.1411;
    public static double kA = 0;
    // public final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(1,1));
    // public final Mass     WEIGHT = Pounds.of(1);
    // public final Distance DIAMETER = Inches.of(2);
    public static AngularVelocity UPPER_SOFT_LIMIT = RPM.of(6500);

  }



  


  
  public static class COMMAND_TRAIN_CONSTANTS{
  
    public static final Angle DOWN_ANGLE = Degrees.of(0);
    public static final Angle SHOOT_ANGLE = Degrees.of(60);
    public static final Angle SAFE_ANGLE = Degrees.of(120);

    public static class INTAKING_COMMAND_CONSTANTS {
      public static double INTAKE_INTAKE_SPEED = -1;
      public static double HOPPER_INTAKE_SPEED = -0.1;
    }

    public static class MIXER_COMMAND_CONSTANTS{
      public static double HOPPER_OUT = 1;
      public static double HOPPER_IN = -1;
      public static double INDEXER_OUT = 1;
    }

    public static class THROWUP_COMMAND_CONSTANTS{
      public static double INTAKE_OUT = 0.75;
      public static double HOPPER_OUT = 0.75;
      public static double INDEXER_OUT = 0.75;
      public static final AngularVelocity SHOOTER_OUT = RPM.of(-500);
    }
    public static class SHOOTER_SPEED{
      public static final AngularVelocity SIDE_TRENCH_VELOCITY = RPM.of(2600);
      public static final AngularVelocity CORRNER_VELOCITY = RPM.of(3000);
      public static final AngularVelocity FAR_VELOCITY = RPM.of(3700);
      public static final AngularVelocity SHORTER_VELOCITY = RPM.of(2800);
    }
  }
}

