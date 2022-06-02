// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public TalonFX leftClimber = new TalonFX(Constants.CLIMBER_LEFT_MOTOR, "Drivetrain");
    public TalonFX rightClimber = new TalonFX(Constants.CLIMBER_RIGHT_MOTOR, "Drivetrain");

    public Climber() {
        rightClimber.setInverted(true);

        leftClimber.setNeutralMode(NeutralMode.Brake);
        rightClimber.setNeutralMode(NeutralMode.Brake);

        leftClimber.setSelectedSensorPosition(0);
        rightClimber.setSelectedSensorPosition(0);

        leftClimber.configReverseSoftLimitThreshold(0);
        leftClimber.configReverseSoftLimitEnable(true);

        rightClimber.configReverseSoftLimitThreshold(0);
        rightClimber.configReverseSoftLimitEnable(true);

        rightClimber.configForwardSoftLimitThreshold(Constants.CLIMB_ENCODER_LIMIT);
        rightClimber.configForwardSoftLimitEnable(true);
  
        leftClimber.configForwardSoftLimitThreshold(Constants.CLIMB_ENCODER_LIMIT);
        leftClimber.configForwardSoftLimitEnable(true);

        leftClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        rightClimber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        leftClimber.config_kP(0, 0.8);
        rightClimber.config_kP(0, 0.8);

    }

    public void setLeftOutput(double percent) {
        leftClimber.set(ControlMode.PercentOutput, percent);
    }

    public void setRightOutput(double percent) {
        rightClimber.set(ControlMode.PercentOutput, percent);
    }

    public void setClimberPosition(double encoderPos) {
        leftClimber.set(ControlMode.Position, encoderPos);
        rightClimber.set(ControlMode.Position, encoderPos);
    }

    public void setClimbEncoders(double value) {
        leftClimber.setSelectedSensorPosition(value);
        rightClimber.setSelectedSensorPosition(value);
    }
    
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Climb encoder right", rightClimber.getSelectedSensorPosition());
        SmartDashboard.putNumber("Climb encoder left", leftClimber.getSelectedSensorPosition());
    }
}