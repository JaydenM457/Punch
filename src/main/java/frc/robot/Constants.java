// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import swervelib.math.Matter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
  
// Shot geometry - tune these
public static final double SHOOTER_HEIGHT_METERS = 0.72;
public static final double HUB_CENTER_HEIGHT_METERS = 2.10;
public static final double SHOT_LAUNCH_ANGLE_DEG = 52.0;

// Release prediction / motion compensation
public static final double RELEASE_LOOKAHEAD_SECS = 0.060;
public static final double MEASURED_VELOCITY_WEIGHT = 0.70;

public static final double HUB_MAX_OMEGA_RAD_PER_SEC = 8.0;
public static final double HUB_HEADING_TOLERANCE_DEG_WITH_TAG = 1.0;
public static final double HUB_HEADING_TOLERANCE_DEG_NO_TAG = 2.0;

public static final double NOTE_EXIT_MPS_PER_RPM = 0.00210;
public static final double NOTE_EXIT_MPS_OFFSET = 1.90;

// Optional clamp
public static final double MIN_SHOOTER_RPM = 1000.0;
public static final double MAX_SHOOTER_RPM = 5200.0;

public static double HUB_THETA_KP = 0.03;
public static double HUB_THETA_KI = 0.0;
public static double HUB_THETA_KD = 0.0025;

public static double noteExitSpeedFromRPM(double rpm) {
  return NOTE_EXIT_MPS_OFFSET + NOTE_EXIT_MPS_PER_RPM * rpm;
}

public static double rpmFromNoteExitSpeed(double speedMps) {
  double rpm = (speedMps - NOTE_EXIT_MPS_OFFSET) / NOTE_EXIT_MPS_PER_RPM;
  return Math.max(MIN_SHOOTER_RPM, Math.min(MAX_SHOOTER_RPM, rpm));
}

public static final InterpolatingDoubleTreeMap SHOT_RPM_MAP = new InterpolatingDoubleTreeMap();
public static final InterpolatingDoubleTreeMap SHOT_TOF_MAP = new InterpolatingDoubleTreeMap();

static {
  // distance (m) -> shooter RPM
  SHOT_RPM_MAP.put(1.8, 2600.0);
  SHOT_RPM_MAP.put(1.9, 2700.0);
  SHOT_RPM_MAP.put(2.0, 2800.0);
  SHOT_RPM_MAP.put(2.1, 2900.0);
  SHOT_RPM_MAP.put(2.2, 3000.0);
  SHOT_RPM_MAP.put(2.5, 3300.0);
  SHOT_RPM_MAP.put(2.8, 3600.0);
  SHOT_RPM_MAP.put(2.9, 3700.0);
  SHOT_RPM_MAP.put(3.0, 3800.0);
  SHOT_RPM_MAP.put(3.1, 3900.0);
  // distance (m) -> NOTE FLIGHT TIME AFTER RELEASE (s)
  SHOT_TOF_MAP.put(1.0, 0.18);
  SHOT_TOF_MAP.put(1.5, 0.23);
  SHOT_TOF_MAP.put(2.0, 0.29);
  SHOT_TOF_MAP.put(2.5, 0.35);
  SHOT_TOF_MAP.put(3.0, 0.42);
  SHOT_TOF_MAP.put(3.5, 0.49);
  SHOT_TOF_MAP.put(4.0, 0.57);
}

public static double getShotRPM(double distanceMeters) {
  return SHOT_RPM_MAP.get(distanceMeters);
}

public static double getShotTimeOfFlight(double distanceMeters) {
  return SHOT_TOF_MAP.get(distanceMeters);
}


public static final Rotation2d FERRY_LEFT_CORNER_HEADING_BLUE = Rotation2d.fromDegrees(160.0);
public static final Rotation2d FERRY_RIGHT_CORNER_HEADING_BLUE = Rotation2d.fromDegrees(-160.0);

public static final Rotation2d FERRY_LEFT_CORNER_HEADING_RED = Rotation2d.fromDegrees(160);
public static final Rotation2d FERRY_RIGHT_CORNER_HEADING_RED = Rotation2d.fromDegrees(-160);

public static final double FERRY_HEADING_TOLERANCE_DEG = 2.0;
public static final double FERRY_SHOOTER_READY_FRACTION = 0.97;

public enum FerrySide {
  LEFT_BACK,
  RIGHT_BACK
}

public static final double FIELD_LENGTH_METERS = 8.1026;

/**
 * Distance from the alliance wall / driver station side to the robot, in meters.
 * Assumes blue alliance is at X = 0 and red alliance is at X = FIELD_LENGTH_METERS.
 */
public static double getDistanceFromDriverStationWall(double robotX, boolean isRedAlliance) {
  return isRedAlliance
      ? FIELD_LENGTH_METERS - robotX
      : robotX;
}

public static final InterpolatingDoubleTreeMap FERRY_RPM_MAP = new InterpolatingDoubleTreeMap();

static {
  // distance from driverstation wall -> shooter rpm
  // EXAMPLE VALUES ONLY, tune these on robot
  FERRY_RPM_MAP.put(1.0, 2600.0);
  FERRY_RPM_MAP.put(2.0, 3000.0);
  FERRY_RPM_MAP.put(3.0, 3350.0);
  FERRY_RPM_MAP.put(4.0, 3650.0);
  FERRY_RPM_MAP.put(5.0, 3950.0);
  FERRY_RPM_MAP.put(6.0, 4250.0);
}
public static final InterpolatingDoubleTreeMap HubRPM = new InterpolatingDoubleTreeMap();

static {
  HubRPM.put(1.8, 2600.0);
  HubRPM.put(1.9, 2700.0);
  HubRPM.put(2.0, 2800.0);
  HubRPM.put(2.1, 2900.0);
  HubRPM.put(2.2, 3000.0);
  HubRPM.put(2.5, 3300.0);
  HubRPM.put(2.8, 3600.0);
  HubRPM.put(2.9, 3700.0);
  HubRPM.put(3.0, 3800.0);
  HubRPM.put(3.1, 3900.0);
}

public static double getHubRPM(double distanceFromHubMeters) {
  return HubRPM.get(distanceFromHubMeters);
}
public static double getFerryRPM(double distanceFromDriverStationMeters) {
  return FERRY_RPM_MAP.get(distanceFromDriverStationMeters);
}



  public static double estimateBallSpeedMps(double shooterRPM) {
  return 3.25 + 0.00205 * shooterRPM;
}

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(13.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // AprilTag IDs for Aiming at Hub
  // The "right" or "left" of the hub is from the perspective of the blue or red alliance's zone.
  
  // These are the IDs of the AprilTags on the sides of the hubs (closest to the robot).
  public static final int blueZoneHubRightTagID = 27;
  public static final Translation2d hubCenterTranslationBlueRight =
    new Translation2d(0.39793333, -0.5969); // example only, replace with actual hub center

  public static final int blueZoneHubLeftTagID = 24;
  public static final Translation2d hubCenterTranslationBlueLeft =
    new Translation2d(0.39793333, -0.5969); // example only, replace with actual hub center

  public static final int redZoneHubRightTagID = 11;
  public static final Translation2d hubCenterTranslationRedRight =
    new Translation2d(0.39793333, -0.5969); // example only, replace with actual hub center

  public static final int redZoneHubLeftTagID = 8;
  public static final Translation2d hubCenterTranslationRedLeft =
    new Translation2d(0.39793333, -0.5969); // example only, replace with actual hub center


  

public static final double noteEffectiveSpeedMps = 9.5; // tune on robot
  // These are the IDs of the AprilTags directly in the center of the hub in the blue or red alliance's zone (not in the neutral zone).
  public static final int blueZoneHubCenterTagID = 26;
  public static final Translation2d hubCenterTranslationBlueCenter =
    new Translation2d(0, -0.5969); // example only, replace with actual hub center

  public static final int blueZoneHubCenterLeftTagID = 25;
  public static final Translation2d hubCenterTranslationBlueCenterLeft =
    new Translation2d(0.39793333, -0.5969); // example only, replace with actual hub center

  public static final int redZoneHubCenterTagID = 10;
  public static final Translation2d hubCenterTranslationRedCenter =
    new Translation2d(0, -0.5969); // example only, replace with actual hub center

  public static final int redZoneHubCenterLeftTagID = 9;
  public static final Translation2d hubCenterTranslationRedCenterLeft =
    new Translation2d(0.39793333, -0.5969); // example only, replace with actual hub center


  public static final int blueZoneOutpostCenterTagID = 29;
  public static final int redZoneOutpostCenterTagID = 13;
 
  public static final double aprilTagAimingPID_kP = 1.0;
  public static final double aprilTagAimingPID_kI = 0.0;
  public static final double aprilTagAimingPID_kD = 0.0;

  public static final int[] HUB_TAG_IDS = {
  blueZoneHubRightTagID,
  blueZoneHubLeftTagID,
  redZoneHubRightTagID,
  redZoneHubLeftTagID,
  blueZoneHubCenterTagID,
  blueZoneHubCenterLeftTagID,
  redZoneHubCenterTagID,
  redZoneHubCenterLeftTagID
};

public static final double SHOT_POSE_PREDICTION_SECS = 0.08;
public static final double SHOT_MEASURED_VELOCITY_WEIGHT = 0.65;
public static final double SHOOTER_READY_FRACTION = 0.97;
public static final double HUB_HEADING_TOLERANCE_DEG_WITH_VISION = 1.25;
public static final double HUB_HEADING_TOLERANCE_DEG_NO_VISION = 2.25;


  public static final Translation2d getHubCenterOffsetFromTag(int tagId) {
  switch (tagId) {
    case blueZoneHubRightTagID:
      return hubCenterTranslationBlueRight;
    case blueZoneHubLeftTagID:
      return hubCenterTranslationBlueLeft;
    case redZoneHubRightTagID:
      return hubCenterTranslationRedRight;
    case redZoneHubLeftTagID:
      return hubCenterTranslationRedLeft;
    case blueZoneHubCenterTagID:
      return hubCenterTranslationBlueCenter;
    case blueZoneHubCenterLeftTagID:
      return hubCenterTranslationBlueCenterLeft;
    case redZoneHubCenterTagID:
      return hubCenterTranslationRedCenter;
    case redZoneHubCenterLeftTagID:
      return hubCenterTranslationRedCenterLeft;
    default:
      throw new IllegalArgumentException("Unknown hub tag ID: " + tagId);
  }
}


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

    public static class INDEXER_CONSTANTS{
    public static final int INDEXER_ID = 4;//SF
  }

  public static class INTAKE_CONSTANTS{
    public static final int INTAKE_ID = 7;//SM
  }

  public static class HOPPER_CONSTANTS{
    public static final int HOPPER_ID = 5;//SM
  }


  public static class ARM_CONSTANTS{
    public static final int ARM_ID = 6;//SM

    public static double kP = 20;
    public static double kD = 0;
    public static double kI = 0;

    public static double kS = 0;
    public static double kG = 0.005;
    public static double kV = 0.01;
    public static double kA = 0; 

    public static Angle LOWER_SOFT_LIMIT = Rotations.of(0.171);
    public static Angle UPPER_SOFT_LIMIT = Rotations.of(0.555);

    public static Angle LOWER_HARD_LIMIT = Rotations.of(0.171);
    public static Angle UPPER_HARD_LIMIT = Rotations.of(0.555);

    public static Angle HORIZONTAL_ZERO = Degrees.of(0.481);
    // public final static Mass     WEIGHT = Pounds.of(3);
    // public final static DCMotor  MOTORS = DCMotor.getNEO(1);
    // public final static Distance LENGTH = Inches.of(14);
    // public final static MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(75));
    public static Angle TOLERANCE = Degrees.of(5);

  }





  public static class SHOOTER_CONSTANTS{
    public static final int SHOOTER_ID = 2;//SF
    public static final int SHOOTER_TWO_ID = 3;//SF

    public static double kP = 0.0128;
    public static double kD = 0;
    public static double kI = 0;

    public static double kS = 0.15;
    public static double kV = 0.1411;
    public static double kA = 0;
    // public final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(1,1));
    // public final Mass     WEIGHT = Pounds.of(1);
    // public final Distance DIAMETER = Inches.of(2);
    public static AngularVelocity LOWER_SOFT_LIMIT = RPM.of(6500);

  }



  


  
  public static class COMMAND_TRAIN_CONSTANTS{
  
    public static final Angle DOWN_ANGLE = Degrees.of(148);
    public static final Angle SHOOT_ANGLE = Degrees.of(60);
    public static final Angle SAFE_ANGLE = Degrees.of(40);

    public static class INTAKING_COMMAND_CONSTANTS {
      public static double INTAKE_INTAKE_SPEED = -1;
      public static double HOPPER_INTAKE_SPEED = -0.1;
    }

    public static class MIXER_COMMAND_CONSTANTS{
      public static double HOPPER_OUT = 1;
      public static double HOPPER_IN = -1;
    }

    public static class THROWUP_COMMAND_CONSTANTS{
      public static double INTAKE_OUT_HALF = 0.5;
      public static double HOPPER_OUT_HALF = 0.5;
      public static double INDEXER_OUT_HALF = 0.5;
      public static final AngularVelocity SHOOTER_OUT = RPM.of(-500);
    }
    public static class SHOOTER_SPEED{
      public static final AngularVelocity SIDE_TRENCH_VELOCITY = RPM.of(2600);
      public static final AngularVelocity CORRNER_VELOCITY = RPM.of(3000);
      public static final AngularVelocity FAR_VELOCITY = RPM.of(3700);
      public static final AngularVelocity SHORTER_VELOCITY = RPM.of(2200);
    }
  }
}

