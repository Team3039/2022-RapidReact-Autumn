// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public CANSparkMax leftClimber;
    public CANSparkMax rightClimber;

    public SparkMaxPIDController leftController;
    public SparkMaxPIDController rightController;
 
    public RelativeEncoder leftEncoder;
    public RelativeEncoder rightEncoder;

    public Climber() {
        leftClimber = new CANSparkMax(Constants.CLIMBER_LEFT_MOTOR, MotorType.kBrushless);
        rightClimber = new CANSparkMax(Constants.CLIMBER_RIGHT_MOTOR, MotorType.kBrushless);

        leftController = leftClimber.getPIDController();
        rightController = rightClimber.getPIDController();

        leftClimber.setInverted(true);
        rightClimber.setInverted(false);
    
        leftEncoder = leftClimber.getEncoder();
        rightEncoder = rightClimber.getEncoder();

        leftClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.setIdleMode(IdleMode.kBrake);

        setClimbEncoders(0);  

        toggleSoftLimits(true);

        leftClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
        leftClimber.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.CLIMB_ENCODER_LIMIT);
    
        rightClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
        rightClimber.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.CLIMB_ENCODER_LIMIT);


        leftController.setP(0.8);
        leftController.setI(0);
        leftController.setD(0);

        rightController.setP(0.8);
        rightController.setI(0);
        rightController.setD(0);
    }

    public static double encoderToRotations(double value) {
        return value / Constants.ENCODER_TO_ROTATIONS_RATIO_NEO;
    }

    public static double rotationsToEncoder(double value) {
        return value * Constants.ENCODER_TO_ROTATIONS_RATIO_NEO;
    }

    public void setLeftOutput(double percent) {
        leftClimber.set(percent);
    }

    public void setRightOutput(double percent) {
        rightClimber.set(percent);
    }

    public void setLeftClimberPosition(double encoderPos) {
        leftController.setReference(encoderToRotations(encoderPos), ControlType.kPosition);
    }

    public void setRightClimberPosition(double encoderPos) {
        rightController.setReference(encoderToRotations(encoderPos), ControlType.kPosition);
    }

    public void setClimbEncoders(double value) {
        leftEncoder.setPosition(value);
        rightEncoder.setPosition(value);
    }

    // ticks
    public double getLeftClimberPosition() {
        return rotationsToEncoder(leftEncoder.getPosition());
    }

    // ticks
    public double getRightClimberPosition() {
        return rotationsToEncoder(rightEncoder.getPosition());
    }

    public void toggleSoftLimits(boolean isOn) {
        leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, isOn);
        leftClimber.enableSoftLimit(SoftLimitDirection.kForward, isOn);     
        rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, isOn);
        rightClimber.enableSoftLimit(SoftLimitDirection.kForward, isOn);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb encoder right", rightEncoder.getPosition());
        SmartDashboard.putNumber("Climb encoder left", leftEncoder.getPosition());
    }
}
       