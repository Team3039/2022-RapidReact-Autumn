// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.AutoForward;
import frc.robot.auto.commands.AutoPop;
import frc.robot.auto.commands.AutoShoot;
import frc.robot.auto.commands.AutoStop;
import frc.robot.auto.commands.AutoStopShooting;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAuto extends SequentialCommandGroup {
  /** Creates a new OneBallAuto. */
  public OneBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(),
      new AutoForward(),
      new WaitCommand(2),
      new AutoStop(),
      new WaitCommand(1),
      new AutoPop(true),
      new WaitCommand(.2),
      new AutoPop(false),
      new AutoStopShooting()
      );
  }
}
