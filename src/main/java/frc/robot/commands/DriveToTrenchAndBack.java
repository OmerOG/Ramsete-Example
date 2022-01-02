// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.JsonTrajectory;
import frc.robot.interfaces.IRamseteDrivetrain;

public class DriveToTrenchAndBack extends SequentialCommandGroup {
  public DriveToTrenchAndBack(IRamseteDrivetrain drivetrain) {
    addCommands
    (
      new RamseteCommand(drivetrain, JsonTrajectory.GET_ALL_TRENCH_BALLS.getTrajectoryFromJson()),
      new RamseteCommand(drivetrain, JsonTrajectory.RETURN_TO_SHOOT.getTrajectoryFromJson(), false)
    );
  }
}
