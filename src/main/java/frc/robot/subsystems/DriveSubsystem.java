// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.DriveConstants.kLeftMasterPort);
  private final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.DriveConstants.kLeftFollowerPort);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.DriveConstants.kRightMasterPort);
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.DriveConstants.kRightFollowerPort);

  private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(leftMaster, leftFollower);
  private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(rightMaster, rightFollower);

  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    leftMaster.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightFollower.configFactoryDefault();

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    leftMaster.configAllSettings(configs);
    leftFollower.configAllSettings(configs);
    rightMaster.configAllSettings(configs);
    rightFollower.configAllSettings(configs);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftMaster.setInverted(Constants.DriveConstants.kLeftInvertType);
    leftFollower.setInverted(Constants.DriveConstants.kLeftInvertType);
    rightMaster.setInverted(Constants.DriveConstants.kRightInvertType);
    rightFollower.setInverted(Constants.DriveConstants.kRightInvertType);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void driveStraight(double fwd, double angleSetPoint) {
    double turningValue = (angleSetPoint - m_gyro.getAngle()) * DriveConstants.kP;
    turningValue = Math.copySign(turningValue, fwd);
    drive.arcadeDrive(fwd, turningValue);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoderDistance() {
    return leftMaster.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoderDistance() {
    return rightMaster.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
