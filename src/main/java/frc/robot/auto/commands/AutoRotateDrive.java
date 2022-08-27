// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoRotateDrive extends CommandBase {
  /** Creates a new AutoRotateDrive. */
  double angle;
  public AutoRotateDrive(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drivetrain.setRotation(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.drivetrain.gyro.getFusedHeading() - angle <= 15 &&
            RobotContainer.drivetrain.gyro.getFusedHeading() - angle >= -15);

  }
}
