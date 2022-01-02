// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.interfaces.IRamseteDrivetrain;

public class Drivetrain extends SubsystemBase implements IRamseteDrivetrain {
  private final WPI_TalonFX _masterLeftController;
  private final WPI_TalonFX _masterRightController;
  private final WPI_TalonFX _slaveLeftController;
  private final WPI_TalonFX _slaveRightController;
  private final AHRS _gyro;
  private final DifferentialDriveOdometry _odometry;
  private final DifferentialDriveKinematics _kinematics;
  private final SimpleMotorFeedforward _motorFeedforward;
  private final RamseteController _ramseteController;

  public Drivetrain() {
    _masterLeftController = new WPI_TalonFX(DrivetrainConstants.FRONT_LEFT_MOTOR_PORT);
    _masterRightController = new WPI_TalonFX(DrivetrainConstants.FRONT_RIGHT_MOTOR_PORT);
    _slaveLeftController = new WPI_TalonFX(DrivetrainConstants.BACK_LEFT_MOTOR_PORT);
    _slaveRightController = new WPI_TalonFX(DrivetrainConstants.BACK_RIGHT_MOTOR_PORT);
    _gyro = new AHRS(Port.kMXP);
    _odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    _kinematics = new DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH_METERS);
    _motorFeedforward = new SimpleMotorFeedforward(DrivetrainConstants.KS_VOLTS,
        DrivetrainConstants.KV_VOLT_SECONDS_PER_METER, DrivetrainConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);
    _ramseteController = new RamseteController();

    configControllers();
  }

  @Override
  public void periodic() {
    _odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
  }

  @Override
  public void resetPose(Pose2d poseOnField) {
    _odometry.resetPosition(poseOnField, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public Pose2d getCurrentPose() {
    return _odometry.getPoseMeters();
  }

  @Override
  public void setLeftVelocity(double feedforward, double velocity) {
    setVelocity(_masterLeftController, feedforward, velocity);
  }

  @Override
  public void setRightVelocity(double feedforward, double velocity) {
    setVelocity(_masterRightController, feedforward, velocity);
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return _kinematics;
  }

  @Override
  public SimpleMotorFeedforward getMotorFeedforward() {
    return _motorFeedforward;
  }

  @Override
  public RamseteController getRamseteController() {
    return _ramseteController;
  }

  private void configControllers() {
    _masterLeftController.configFactoryDefault();
    _masterRightController.configFactoryDefault();
    _slaveRightController.configFactoryDefault();
    _slaveLeftController.configFactoryDefault();

    _slaveRightController.follow(_masterRightController);
    _slaveLeftController.follow(_masterLeftController);

    _masterRightController.setInverted(InvertType.InvertMotorOutput);
  }

  private void setVelocity(WPI_TalonFX motor, double feedforward, double velocity) {
    /**
     * In this motor controller, arbitrary feedforward range is [-1, 1], while
     * closed loop output units are [-1023, 1023]. This is known by CTRE and should
     * be fixed to [-1, 1] as well in later versions. Until the fix, this weird unit
     * range is taken into account when calculating the velocity for closed loop
     * control.
     */
    double feedforwardInPercent = feedforward / motor.getBusVoltage();
    double velocityInClosedLoopUnits = convertVelocityToTalonFXClosedLoopUnits(velocity);

    motor.set(ControlMode.Velocity, velocityInClosedLoopUnits, DemandType.ArbitraryFeedForward, feedforwardInPercent);
  }

  private double getHeading() {
    return Math.IEEEremainder(_gyro.getAngle(), 360) * (-1.0); // [-180, 180] degrees clockwise
  }

  private double getLeftDistance() {
    return encoderUnitsToMeters(_masterLeftController.getSelectedSensorPosition());
  }

  private double getRightDistance() {
    return encoderUnitsToMeters(_masterRightController.getSelectedSensorPosition());
  }

  private double encoderUnitsToMeters(double encoderUnits) {
    return encoderUnits / DrivetrainConstants.GEAR_RATIO / DrivetrainConstants.ENCODER_COUNTS_PER_REVOLUTION
        * DrivetrainConstants.WHEEL_CIRCUMFRERENCE_METERS;
  }

  private double convertVelocityToTalonFXClosedLoopUnits(double metersPerSecond) {
    return metersPerSecond * DrivetrainConstants.GEAR_RATIO * DrivetrainConstants.ENCODER_COUNTS_PER_REVOLUTION
        / DrivetrainConstants.WHEEL_CIRCUMFRERENCE_METERS / 1023 / 10;
  }
}
