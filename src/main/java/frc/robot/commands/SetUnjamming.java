// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;

public class SetUnjamming extends CommandBase {
  /** Creates a new SetUnjamming. */
  public SetUnjamming() {
   addRequirements(RobotContainer.indexer);
   addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   RobotContainer.indexer.unjam();
  //  RobotContainer.intake.setRollerMotor(-.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   RobotContainer.intake.setRollerMotor(0);
   RobotContainer.indexer.setFrontMotor(0);
   RobotContainer.indexer.setBackMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
