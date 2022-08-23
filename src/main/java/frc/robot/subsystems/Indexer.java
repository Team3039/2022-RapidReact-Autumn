// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
 public TalonSRX frontWheelMotor = new TalonSRX(Constants.FRONT_INDEXER_WHEEL);
 public TalonSRX backWheelMotor = new TalonSRX(Constants.BACK_INDEXER_WHEEL);

 DigitalInput topGate = new DigitalInput(Constants.TOP_BEAM_BREAK);
 DigitalInput bottomGate = new DigitalInput(Constants.BOTTOM_BEAM_BREAK);

public Indexer() {
 frontWheelMotor.setNeutralMode(NeutralMode.Brake);
 backWheelMotor.setNeutralMode(NeutralMode.Brake);
 }

public void setFrontMotor(double frontMotorOutput) {
 frontWheelMotor.set(ControlMode.PercentOutput, frontMotorOutput);
 }

public void setBackMotor(double backMotorOutput) {
 backWheelMotor.set(ControlMode.PercentOutput, backMotorOutput);
 }

public void indexCargo() {
 if(topGate.get()) {
  setFrontMotor(.25);
  setBackMotor(.25);
 }
 if((!topGate.get() && bottomGate.get())) {
  setFrontMotor(.25);
  setBackMotor(0);
  } else {
   setFrontMotor(0);
   setBackMotor(0);
  }
 }

public void unjam() {
 setFrontMotor(-.35);
 setBackMotor(-.35);
 }

@Override 
public void periodic() {
// This method will be called once per scheduler run
SmartDashboard.putBoolean("First Stage Beam Break", bottomGate.get());
SmartDashboard.putBoolean("Second Stage Beam Break", topGate.get());
 }
}