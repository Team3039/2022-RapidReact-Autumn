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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  
  CANSparkMax rollerMotor = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
  TalonSRX actuateMotor = new TalonSRX(Constants.INTAKE_ACTUATE_MOTOR);

  DigitalInput limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH);
  
  public boolean isIntakeActuated = true;
  public boolean isIntakeManual = false;

  // Negative voltage brings intake up, positive voltage brings intake down
  public Intake() {
   actuateMotor.setInverted(true);
   rollerMotor.setIdleMode(IdleMode.kCoast);
   actuateMotor.setNeutralMode(NeutralMode.Brake);

   actuateMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  //  actuateMotor.setSelectedSensorPosition(0);
   actuateMotor.setSensorPhase(true);

  //  yes we know there is this cool thing called feed fwd, WE DONT HAVE TIME
   // down 
   actuateMotor.config_kP(0, 0.35);
   actuateMotor.config_kI(0, 0);
   actuateMotor.config_kD(0, 4.5);

   // up
   actuateMotor.config_kP(1, 0.4);
   actuateMotor.config_kI(1, 0);
   actuateMotor.config_kD(1, 4);

   actuateMotor.configForwardSoftLimitEnable(true);
   actuateMotor.configReverseSoftLimitEnable(true);
   actuateMotor.configForwardSoftLimitThreshold(1700);
   actuateMotor.configReverseSoftLimitThreshold(0);
  }

  public void setRollerMotor(double RollerMotorSpeed) {
   rollerMotor.set(RollerMotorSpeed);
  }

  public void setActuateMotor(boolean isActuated) {
   if (isActuated) {
   actuateMotor.selectProfileSlot(0, 0);
   actuateMotor.set(ControlMode.Position, 1340);
   }
   else {
   actuateMotor.selectProfileSlot(1, 0);
   actuateMotor.set(ControlMode.Position, 0);
   }
   SmartDashboard.putNumber("Intake Actuate Error", actuateMotor.getClosedLoopError());
  }

  public void manualIntake() {
    actuateMotor.set(ControlMode.PercentOutput, RobotContainer.getOperator().getLeftYAxis() * 0.15);
  }

  public void limitSwitchEncoderReset() {
    if (!limitSwitch.get()) {
      actuateMotor.setSelectedSensorPosition(0);
    }
  }

  @Override
  public void periodic() {
    if (!isIntakeManual) {
    setActuateMotor(isIntakeActuated);
    }
    else {
    manualIntake();
    }

    limitSwitchEncoderReset();

    SmartDashboard.putNumber("intake encoder", actuateMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("intake limit switch", limitSwitch.get());
    
  }
}
