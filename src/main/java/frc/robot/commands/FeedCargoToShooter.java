// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class FeedCargoToShooter extends CommandBase {
  /** Creates a new FeedCargoToShooter. */
  double frontSpeed;
  double backSpeed;

  public FeedCargoToShooter(double frontSpeed, double backSpeed) {
   addRequirements(RobotContainer.indexer);
   this.frontSpeed = frontSpeed;
   this.backSpeed = backSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   RobotContainer.indexer.setBackMotor(backSpeed);
   RobotContainer.indexer.setFrontMotor(frontSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   RobotContainer.indexer.setBackMotor(0);
   RobotContainer.indexer.setFrontMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
