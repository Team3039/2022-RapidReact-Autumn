// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  CANSparkMax rollerMotor = new CANSparkMax(Constants.INTAKE_ROLLER_MOTOR, MotorType.kBrushless);
  // Solenoid deployMotor = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKEDEPLOYSOLENOID);

  public Intake() {
   rollerMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setRollerMotor(double RollerMotorSpeed) {
   rollerMotor.set(RollerMotorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
