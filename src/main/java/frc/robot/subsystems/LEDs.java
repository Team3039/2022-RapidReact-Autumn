// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  
  DigitalOutput[] outputs = {new DigitalOutput(Constants.LED_PORT_A),
                             new DigitalOutput(Constants.LED_PORT_B)
  };

  public boolean[] isBootingUp = { true, true };

  public boolean[] isAtShooterSetpoint = { false, true };

  public boolean[] isClimbInitiated = { true, false };

  public boolean[] isIdle = { false, false };
  
  public LEDs() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
