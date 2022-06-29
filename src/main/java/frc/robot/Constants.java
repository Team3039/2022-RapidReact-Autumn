// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// Ports

// Indexer
public static final int FRONT_INDEXER_WHEEL = 0;
public static final int BACK_INDEXER_WHEEL = 1;

// Beam breaks
public static final int TOP_BEAM_BREAK = 0;
public static final int BOTTEM_BEAM_BREAK = 1;

// Intake
public static final int INTAKE_ROLLER_MOTOR = 0;
public static final int INTAKE_ACTUATE_MOTOR = 1;

// Shooter
public static final int SHOOTER_LEADER_MOTOR = 11;
public static final int SHOOTER_FOLLOWER_MOTOR = 12;
public static final int SHOOTER_SOLENOID = 1;
public static final double SHOOTER_TO_ENCODER_RATIO = 1;
public static final double TICKS_PER_ROTATION = 2048;

// CLimber 
public static final int CLIMBER_LEFT_MOTOR = 4;
public static final int CLIMBER_RIGHT_MOTOR = 5;
public static final double CLIMB_ENCODER_LIMIT = 130000;
public static final double TELESCOPING_TO_MID_BAR_VALUE_RIGHT = 219000;
public static final double TELESCOPING_TO_MID_BAR_VALUE_LEFT = 200000;

// LEDS 
public static final int LED_PORT_A = 0;
public static final int LED_PORT_B = 1;

// Turret
public static final int TURRET_MOTOR_ID = 0;

// Drive
public static final double DRIVE_Y = .9;
public static final double DRIVE_ROT = .9;
public static final int LEFT_FRONT_MOTOR = 0;
public static final int LEFT_REAR_MOTOR = 1;  
public static final int RIGHT_FRONT_MOTOR = 2; 
public static final int RIGHT_REAR_MOTOR = 3; 

}