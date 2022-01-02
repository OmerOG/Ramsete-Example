// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IRamseteDrivetrain extends Subsystem {
  /**
   * Sets the left velocity of the drivetrain
   * 
   * @param feedforward Pre-calculated power to apply in volts
   * @param velocity    Target velocity in m/s
   */
  public void setLeftVelocity(double feedforward, double velocity);

  /**
   * Sets the right velocity of the drivetrain
   * 
   * @param feedforward Pre-calculated power to apply in volts
   * @param velocity    Target velocity in m/s
   */
  public void setRightVelocity(double feedforward, double velocity);

  /**
   * Sets the current pose of the drivetrain on the field
   */
  public void resetPose(Pose2d poseOnField);

  /**
   * Gets the current pose of the drivetrain on the field
   * 
   * @return Drivetrain pose on the field
   */
  public Pose2d getCurrentPose();

  /**
   * Gets the kinematic model of the drivetrain for calculations
   * 
   * @return Kinematics of the drivetrain
   */
  public DifferentialDriveKinematics getKinematics();

  /**
   * Gets the model of the motor for feedforward calculations
   * 
   * @return Feedforward motor model
   */
  public SimpleMotorFeedforward getMotorFeedforward();

  /**
   * Gets the RAMSETE controller for global pose calculations
   * 
   * @return RAMSETE controller
   */
  public RamseteController getRamseteController();
}
