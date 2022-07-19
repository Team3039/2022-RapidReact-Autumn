// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SpinShooter;

public class Shooter extends SubsystemBase {
  
  TalonFX batmanMotor = new TalonFX(Constants.SHOOTER_LEADER_MOTOR);
  TalonFX robinMotor = new TalonFX(Constants.SHOOTER_FOLLOWER_MOTOR);
  // Servo redHood = new Servo(Constants.SHOOTER_SERVO);

  public static double setPoint = 0;
  public static boolean isAtSetpoint = false;

  // public static InterpolatingTreeMap<InterpolatingDouble, Vector2> shooterMap;

  // private double[] RPMRegressionVariable = {0};
  // private double[] hoodRegressionVariable = {0};

  public Shooter() {
   batmanMotor.setInverted(true);
   batmanMotor.setNeutralMode(NeutralMode.Coast);

   robinMotor.setInverted(false);
   robinMotor.setNeutralMode(NeutralMode.Coast);

   batmanMotor.config_kP(0, .3);
   batmanMotor.config_kI(0, .0002);
   batmanMotor.config_kD(0, 12);

   robinMotor.follow(batmanMotor);

  //  shooterMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>();

  //  shooterMap.put(new InterpolatingDouble(Double.valueOf(-8.3)), new Vector2(2200, 0));
  //  shooterMap.put(new InterpolatingDouble(Double.valueOf(-14)), new Vector2(2600, 0.2));
  //  shooterMap.put(new InterpolatingDouble(Double.valueOf(-17)), new Vector2(2800, 0.4));
  }

  public double velocityToRPM(double velocity) {
   return velocity / Constants.SHOOTER_TO_ENCODER_RATIO / Constants.TICKS_PER_ROTATION * 600;
  }

  public double RPMToVelocity(double rpm) {
   return rpm * Constants.SHOOTER_TO_ENCODER_RATIO * Constants.TICKS_PER_ROTATION / 600;
  }

  public void setShooterRPM(double rpm) {
   batmanMotor.set(ControlMode.Velocity, RPMToVelocity(rpm));
   robinMotor.set(ControlMode.Velocity, RPMToVelocity(rpm));
  }

  public void setShooterPercent(double percent) {
   batmanMotor.set(ControlMode.PercentOutput, percent);
   robinMotor.set(ControlMode.PercentOutput, percent);
  }

  // public void setHoodAngle(double pos) {
  //  hood.setPosition(pos);
  // }

  // public double calculateShooterRPM() {
  //  double output = 0;
  //  for(int i = 1; i < RPMRegressionVariable.length; i++) {
  //   output += Math.pow(RPMRegressionVariable[1], i);
  //  }
  // return output;
  // }

  // public double calculateHoodAngle() {
  //  double output = 0;
  //  for(int i = 1; i < hoodRegressionVaraible.length; i++) {
  //   output += Math.pow(hoodRegressionVaraible[1], i);
  //  }
  //  return output;
  // }

  @Override
  public void periodic() {
   SmartDashboard.putNumber("Shooter SetPoint Velocity", velocityToRPM(batmanMotor.getSelectedSensorVelocity()));
  //  SmartDashboard.putNumber("Hood Angle", hood.getPosition());
  //  batmanMotor.set(ControlMode.Velocity, RPMToVelocity(2400));
  }
}
