// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  TalonFX batmanMotor = new TalonFX(Constants.SHOOTER_LEADER_MOTOR);
  TalonFX robinMotor = new TalonFX(Constants.SHOOTER_FOLLOWER_MOTOR);
  Solenoid batgirlSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SHOOTER_SOLENOID);

  public Shooter() {
  //  batmanMotor.setInverted(true);
   batmanMotor.setNeutralMode(NeutralMode.Coast);

  //  robinMotor.setInverted(false);
   robinMotor.setNeutralMode(NeutralMode.Coast);

   batmanMotor.config_kP(0, 0.9);
   batmanMotor.config_kI(0, 0.00015);
   batmanMotor.config_kD(0, 6);

   robinMotor.follow(batmanMotor);
  }

  public double velocityToRPM(double velocity) {
   return velocity / Constants.SHOOTER_TO_ENCODER_RATIO / Constants.TICKS_PER_ROTATION * 600;
  }

  public double RPMToVelocity(double rpm) {
   return rpm * Constants.SHOOTER_TO_ENCODER_RATIO * Constants.TICKS_PER_ROTATION / 600;
  }

  public void setShooterRPM(double rpm) {
   batmanMotor.set(ControlMode.Velocity, RPMToVelocity(rpm));
  }

  public void setShooterPercent(double percent) {
   batmanMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setBatgirl(boolean batgirlsWheelchair) {
   batgirlSolenoid.set(batgirlsWheelchair);
  }

  @Override
  public void periodic() {
   SmartDashboard.putNumber("Shooter SetPoint Velocity", velocityToRPM(batmanMotor.getSelectedSensorVelocity()));
  //  SmartDashboard.putNumber("Hood Angle", hood.getPosition());
  }
}