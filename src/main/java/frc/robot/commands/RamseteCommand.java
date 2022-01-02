// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.interfaces.IRamseteDrivetrain;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to
 * follow a trajectory {@link Trajectory} with a differential drive.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommand extends CommandBase {
    private final Trajectory _trajectory;
    private final IRamseteDrivetrain _drivetrain;
    private final DifferentialDriveKinematics _kinematics;
    private final SimpleMotorFeedforward _motorFeedforward;
    private final RamseteController _follower;
    private final Timer _timer;
    private final double _previousTime;
    private final DifferentialDriveWheelSpeeds _previousWheelSpeeds;
    private final boolean _resetToInitialPose;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided
     * trajectory.
     *
     * @param trajectory The trajectory to follow.
     * @param drivetrain Drivetrain that supports RAMSETE
     */
    public RamseteCommand(IRamseteDrivetrain drivetrain, Trajectory trajectory) {
        this(drivetrain, trajectory, true);
    }

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided
     * trajectory.
     *
     * @param trajectory         The trajectory to follow.
     * @param drivetrain         Drivetrain that supports RAMSETE
     * @param resetToInitialPose Reset the robot position on the field to the
     *                           initial pose of the trajectory. Pass false only if
     *                           you are running multiple trajectories in a row and
     *                           the end pose of the previous trajectory
     *                           overlaps/very close to this one's initial pose (to
     *                           fix small accumulated error). Make sure you know
     *                           what you're doing!
     */
    public RamseteCommand(IRamseteDrivetrain drivetrain, Trajectory trajectory, boolean resetToInitialPose) {
        _trajectory = trajectory;
        _drivetrain = drivetrain;
        _kinematics = _drivetrain.getKinematics();
        _motorFeedforward = _drivetrain.getMotorFeedforward();
        _follower = _drivetrain.getRamseteController();
        _timer = new Timer();
        _previousTime = 0;
        _previousWheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
        _resetToInitialPose = resetToInitialPose;

        addRequirements(_drivetrain);
    }

    @Override
    public void initialize() {
        if (_resetToInitialPose) {
            _drivetrain.resetPose(_trajectory.getInitialPose());
        }

        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        double currentTime = _timer.get();
        double deltaTime = currentTime - _previousTime;

        Pose2d currentPose = _drivetrain.getCurrentPose();
        State desiredState = _trajectory.sample(currentTime);
        ChassisSpeeds targetChassisSpeeds = _follower.calculate(currentPose, desiredState);
        DifferentialDriveWheelSpeeds targetWheelSpeeds = _kinematics.toWheelSpeeds(targetChassisSpeeds);

        double leftFeedforward = calculateFeedforward(deltaTime, targetWheelSpeeds.leftMetersPerSecond,
                _previousWheelSpeeds.leftMetersPerSecond);
        double rightFeedforward = calculateFeedforward(deltaTime, targetWheelSpeeds.rightMetersPerSecond,
                _previousWheelSpeeds.rightMetersPerSecond);

        _drivetrain.setLeftVelocity(leftFeedforward, targetWheelSpeeds.leftMetersPerSecond);
        _drivetrain.setRightVelocity(rightFeedforward, targetWheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        _timer.stop();
        _drivetrain.setLeftVelocity(0, 0);
        _drivetrain.setRightVelocity(0, 0);
    }

    @Override
    public boolean isFinished() {
        return _timer.hasElapsed(_trajectory.getTotalTimeSeconds());
    }

    private double calculateFeedforward(double deltaTime, double speed, double previousSpeed) {
        double acceleration = (speed - previousSpeed) / deltaTime;
        double feedforwardOutput = _motorFeedforward.calculate(speed, acceleration);
        return feedforwardOutput;
    }
}
