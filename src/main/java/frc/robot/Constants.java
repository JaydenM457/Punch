// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

 public static final class AutonConstants
 {

  //  public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //  public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  public static final int aimAtTargetID = 1;
 }

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

    public static class ARM_CONSTANTS{
      public static final int ARM_ID = 6;//SM
  }

    public static class INDEXER_CONSTANTS{
      public static final int INDEXER_ID = 4;//SF
  }

    public static class INTAKE_CONSTANTS{
      public static final int INTAKE_ID = 7;//SM
  }

    public static class SHOOTER_CONSTANTS{
      public static final int SHOOTER_ID = 2;//SF
      public static final int SHOOTER_TWO_ID = 3;//SF
  }

    public static class HOPPER_CONSTANTS{
      public static final int HOPPER_ID = 5;//SM
  }

  public static class COMMAND_TRAIN_CONSTANTS{
    
  }
}

