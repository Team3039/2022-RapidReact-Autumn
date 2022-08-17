// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  CANSparkMax rollerMotor = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
  TalonSRX actuateMotor = new TalonSRX(Constants.INTAKE_ACTUATE_MOTOR);
  
  public boolean isIntakeActuated = false;

  public Intake() {
   rollerMotor.setIdleMode(IdleMode.kCoast);
   actuateMotor.setNeutralMode(NeutralMode.Brake);

   actuateMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

   actuateMotor.config_kP(0, 0.1);
   actuateMotor.config_kI(0, 0);
   actuateMotor.config_kD(0, 0);

   actuateMotor.configForwardSoftLimitEnable(true);
   actuateMotor.configReverseSoftLimitEnable(true);
   actuateMotor.configForwardSoftLimitThreshold(1350);
   actuateMotor.configReverseSoftLimitThreshold(0);
  }

  public void setRollerMotor(double RollerMotorSpeed) {
   rollerMotor.set(RollerMotorSpeed);
  }

  public void setActuateMotor(boolean isActuated) {
   if (isActuated) {
   actuateMotor.set(ControlMode.Position, 1250);
   }
   else {
   actuateMotor.set(ControlMode.Position, 50);
   }
  }

  @Override
  public void periodic() {
    setActuateMotor(isIntakeActuated);

    System.out.println(actuateMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("intake encoder", actuateMotor.getSelectedSensorPosition());
  }
}
