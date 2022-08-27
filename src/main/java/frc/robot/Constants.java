// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// Indexer
public static final int FRONT_INDEXER_WHEEL = 9;
public static final int BACK_INDEXER_WHEEL = 8;

// Beam breaks
public static final int TOP_BEAM_BREAK = 1;
public static final int BOTTOM_BEAM_BREAK = 0;

// Intake
public static final int INTAKE_ROLLER_MOTOR = 7;
public static final int INTAKE_ACTUATE_MOTOR = 0;

public static final int LIMIT_SWITCH = 2;

// Shooter
public static final int SHOOTER_LEADER_MOTOR = 11;
public static final int SHOOTER_FOLLOWER_MOTOR = 12;
public static final double SHOOTER_TO_ENCODER_RATIO = 1;
public static final double TICKS_PER_ROTATION = 2048;

// CLimber 
public static final int CLIMBER_LEFT_MOTOR = 5;
public static final int CLIMBER_RIGHT_MOTOR = 6;
public static final double CLIMB_ENCODER_LIMIT = 80;
public static final double TELESCOPING_TO_MID_BAR_VALUE_RIGHT = 0;
public static final double TELESCOPING_TO_MID_BAR_VALUE_LEFT = 0;
public static final double ENCODER_TO_ROTATIONS_RATIO_NEO = 42;

// LEDS 
public static final int LED_PORT_A = 0;
public static final int LED_PORT_B = 1;

// Turret
public static final int TURRET_MOTOR_ID = 10;
public static final double kP_TURRET_TRACK = .015;

// Drive
 public static final double DRIVE_Y = 0.70;
 public static final double DRIVE_ROT = 0.70;
 public static final int LEFT_FRONT_MOTOR = 1; // RR
 public static final int LEFT_REAR_MOTOR = 2;  //RL
 public static final int RIGHT_FRONT_MOTOR = 4; //LL
 public static final int RIGHT_REAR_MOTOR = 3; //LR

 public static final double AUTO_DRIVE_ROTATE_KP = 0.008;

 // Auto
  // 2020 Drive Constants
  public static final double kWheelDiameterInches = 6;
  public static final double kTrackWidthInches = 29.75;
  public static final double kDriveEncoderPPR = 42;
  public static final double kEncoderRotationToWheelRotationRatio = 8.45/1.0;

  public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);
  public static final DifferentialDriveKinematics kDriveKinematics =
          new DifferentialDriveKinematics(kTrackWidthMeters);

  public static final double ksVolts = 0;
  public static final double kvVoltSecondsPerMeter = 0;
  public static final double kaVoltSecondsSquaredPerMeter = 0;

  public static final double kPDriveVel = 1.5; //8.5
  public static final double kDDriveVel = 0;

  public static final double kMinSpeedMetersPerSecond = Units.feetToMeters(4.5); //Find good value
  public static final double kMinAcclerationMetersPerSecondSquared = Math.pow(Units.feetToMeters(4.5), 2);

  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(10.5); //Find good value
  public static final double kMaxAccelerationMetersPerSecondSquared = Math.pow(Units.feetToMeters(8), 2);

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2.0;
  public static final double kRamseteZeta = 0.7;


}