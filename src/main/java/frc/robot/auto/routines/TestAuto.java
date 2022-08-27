// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.AutoDrive;
import frc.robot.auto.commands.AutoIndexCargo;
import frc.robot.auto.commands.AutoResetOdometry;
import frc.robot.auto.commands.AutoRotateDrive;
import frc.robot.auto.commands.AutoSpinShooter;
import frc.robot.auto.commands.AutoStopIndex;
import frc.robot.auto.commands.RunMotorFrontINdexer;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.auto.commands.Sequences.AutoPathWithIndexing;
import frc.robot.auto.commands.Sequences.AutoShootingSquence;
import frc.robot.subsystems.Drivetrain;

public class TestAuto extends SequentialCommandGroup {

  TrajectoryGenerator trajectories = TrajectoryGenerator.getInstance();
  Drivetrain drive = RobotContainer.drivetrain;

  public TestAuto() {

    // RamseteCommand testPathOne = new RamseteCommand(
    //   trajectories.getTestPath(),
    //   drive::getPose,
    //   new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    //   new SimpleMotorFeedforward(Constants.ksVolts,
    //           Constants.kvVoltSecondsPerMeter,
    //           Constants.kaVoltSecondsSquaredPerMeter),
    //   Constants.kDriveKinematics,
    //   drive::getWheelSpeeds,
    //   new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
    //   new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
    //   // RamseteCommand passes volts to the callback
    //   drive::tankDriveVolts,
    //   drive);

    addCommands(
      new AutoResetOdometry(),   
      new ParallelCommandGroup(  
        new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new WaitCommand(1.8),
          new AutoIndexCargo(), 
          new AutoDrive(0.15, 0.20)
        ),
        new AutoStopIndex(),
        new AutoRotateDrive(180),
        new AutoShootingSquence(),
        new AutoStopIndex()),
      new RunMotorFrontINdexer()    
    
        )

    );
  }
}
