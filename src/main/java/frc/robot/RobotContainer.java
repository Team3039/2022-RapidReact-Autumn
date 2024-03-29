// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DisableClimbSoftLimits;
import frc.robot.commands.FeedCargoToShooter;
import frc.robot.commands.SetIndexingSequence;
import frc.robot.commands.SetLeftClimber;
import frc.robot.commands.SetRightClimber;
import frc.robot.commands.SetUnjamming;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.ToggleManualIntake;
import frc.robot.commands.TrackTarget;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
   public static Climber climber = new Climber();
   public static Shooter shooter = new Shooter();
   public static Indexer indexer = new Indexer();
   public static Intake intake = new Intake();
   public static Turret turret = new Turret();
   public static Drivetrain drivetrain = new Drivetrain();

   public static PS4Gamepad driverPad = new PS4Gamepad(0);
   public static PS4Gamepad operatorPad = new PS4Gamepad(1);

   private final Button driverX = driverPad.getButtonX();
   private final Button driverSquare = driverPad.getButtonSquare();
   private final Button driverTriangle = driverPad.getButtonTriangle();
   private final Button driverCircle = driverPad.getButtonCircle();

   private final Button driverShare = driverPad.getShareButton();
   private final Button driverOptions = driverPad.getOptionsButton();
   private final Button driverPadButton = driverPad.getButtonPad();
   private final Button driverStartButton = driverPad.getStartButton();

   private final Button driverL1 = driverPad.getL1();
   private final Button driverL2 = driverPad.getL2();
   private final Button driverL3 = driverPad.getL3();
   private final Button driverR1 = driverPad.getR1();
   private final Button driverR2 = driverPad.getR2();
   private final Button driverR3 = driverPad.getR3();

   private final Button driverDPadUp = driverPad.getDPadUp();
   private final Button driverDPadDown = driverPad.getDPadDown();
   private final Button driverDPadLeft = driverPad.getDPadLeft();
   private final Button driverDPadRight = driverPad.getDPadRight();


   private final Button operatorTriangle = operatorPad.getButtonTriangle();
   private final Button operatorSquare = operatorPad.getButtonSquare();
   private final Button operatorCircle = operatorPad.getButtonCircle();
   private final Button operatorX = operatorPad.getButtonX();

   private final Button operatorShare = operatorPad.getShareButton();
   private final Button operatorOptions = operatorPad.getOptionsButton();
   private final Button operatorPadButton = operatorPad.getButtonPad();
   private final Button operatorStartButton = operatorPad.getStartButton();

   private final Button operatorL1 = operatorPad.getL1();
   private final Button operatorL2 = operatorPad.getL2();
   private final Button operatorL3 = operatorPad.getL3();
   private final Button operatorR1 = operatorPad.getR1();
   private final Button operatorR2 = operatorPad.getR2();
   private final Button operatorR3 = operatorPad.getR3();
   
   private final Button operatorDPadDown = operatorPad.getDPadDown();
   private final Button operatorDPadUp = operatorPad.getDPadUp();
   private final Button operatorDPaLeft = operatorPad.getDPadLeft();
   private final Button operatorDPadRight = operatorPad.getDPadRight();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    driverL1.whileHeld(new SetLeftClimber(.60));
    driverL2.whileHeld(new SetLeftClimber(-.60));

    driverR1.whileHeld(new SetRightClimber(.60));
    driverR2.whileHeld(new SetRightClimber(-.60));

    
    driverPadButton.toggleWhenPressed(new DisableClimbSoftLimits());
    
    driverOptions.toggleWhenPressed(new TrackTarget());
    
    // Shooter Rotations Per Minute
    operatorR1.whileHeld(new SpinShooter(2700));
    // intake, front indexer wheels, back indexer wheels (0 is neutral, 1 is 100%)
    operatorL1.whileHeld(new SetIndexingSequence(.5, .6, .25));
    // front indexer wheels, back indexer wheels (0 is neutral, 1 is 100%)
    operatorR2.whileHeld(new FeedCargoToShooter(.4, .4));

    operatorL2.whileHeld(new SetUnjamming());
    operatorShare.toggleWhenPressed(new TrackTarget());

    operatorStartButton.toggleWhenPressed(new ToggleManualIntake());
  }

  public static PS4Gamepad getDriver() {
   return driverPad;
  }

  public static PS4Gamepad getOperator() {
   return operatorPad;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
  
    return null;
  }
}