// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  
 public static double targetValid;
 public static double targetX;
 public static double targetY;
 public static double targetArea; 

 public TalonSRX turretMotor = new TalonSRX(Constants.TURRET_MOTOR_ID);

 public static boolean isAtTargetPositon;
  
public Turret() {
 turretMotor.setSelectedSensorPosition(0);
 turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

 turretMotor.setNeutralMode(NeutralMode.Coast);

 turretMotor.configForwardSoftLimitThreshold(degreesToTicks(90)); // Needs to be configured for new turret
 turretMotor.configReverseSoftLimitThreshold(degreesToTicks(-90)); // Needs to be configured for new turret

 turretMotor.configForwardSoftLimitEnable(true);
 turretMotor.configReverseSoftLimitEnable(true);

 turretMotor.config_kP(0, 0);
 turretMotor.config_kI(0, 0);
 turretMotor.config_kD(0, 0);
    
 turretMotor.config_kP(1, 0);
 turretMotor.config_kI(1, 0);
 turretMotor.config_kD(1, 0);
 
 turretMotor.selectProfileSlot(0, 0);
}

public void trackTarget() {
 setTurretPosition(getCurrentAngle() - targetX);
}

public void setTurretPosition(double targetAngle) {
 turretMotor.set(ControlMode.Position, degreesToTicks(targetAngle));
}

public double degreesToTicks(double theta) {
 return theta * (22320 / 360);
}

public double getCurrentAngle() {
 return (turretMotor.getSelectedSensorPosition() / 22320) * 360;
}

public void stop() {
 turretMotor.set(ControlMode.PercentOutput, 0);
}

public void resetEncoder() {
 turretMotor.setSelectedSensorPosition(0);
}

@Override
public void periodic() {
// This method will be called once per scheduler run
 SmartDashboard.putNumber("Current Angle", getCurrentAngle());
 SmartDashboard.putNumber("TargetX", targetX);

 SmartDashboard.putBoolean("Is At Target Position", isAtTargetPositon);

 targetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
 targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
 targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
 targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
 }
}
