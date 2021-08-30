/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveByCommand;
import frc.robot.commands.TurnByCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

public class AutoPath extends SequentialCommandGroup {
  /**
   * Creates a new nlasduh.
   */
  public AutoPath(double xDis, double yDis, boolean turn1, boolean turn2, DrivebaseSubsystem driveAuto) {
    if (xDis != 0) {
      addCommands(new MoveByCommand(xDis, driveAuto, 0));
    }

    if (turn1) {
      addCommands(new TurnByCommand(90, driveAuto, 0));
    }

    if (yDis != 0) {
      addCommands(new MoveByCommand(yDis, driveAuto, 0));
    }

    if (turn2) {
      addCommands(new TurnByCommand(90, driveAuto, 0));
    }

  }
}