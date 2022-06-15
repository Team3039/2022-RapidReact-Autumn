// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  VictorSPX rollerMotor = new VictorSPX(Constants.INTAKE_ROLLER_MOTOR);
  // CANSparkMax rollerMotor = new CANSparkMax(MotorType.kBrushless, Constants.INTAKE_ROLLER_MOTOR);
  TalonSRX actuateMotor = new TalonSRX(Constants.INTAKE_ACTUATE_MOTOR);
  
  // Solenoid deployMotor = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKEDEPLOYSOLENOID);

  public Intake() {
   rollerMotor.setNeutralMode(NeutralMode.Coast);
   actuateMotor.setNeutralMode(NeutralMode.Brake);

   actuateMotor.setSelectedSensorPosition(0);
   actuateMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

   actuateMotor.config_kP(0, 0);
   actuateMotor.config_kI(0, 0);
   actuateMotor.config_kD(0, 0);
  }

  public void setRollerMotor(double RollerMotorSpeed) {
   rollerMotor.set(ControlMode.PercentOutput, RollerMotorSpeed);
  }

  public void setActuateMotor(double degrees) {
   actuateMotor.set(ControlMode.Position, degrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
