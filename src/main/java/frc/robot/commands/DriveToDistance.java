// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToDistance extends PIDCommand {
  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem drive, double distance, double angle) {
    super(
        // The controller that the command will use
        new PIDController(
          DriveConstants.driveStraightGains.kP, 
          DriveConstants.driveStraightGains.kP, 
          DriveConstants.driveStraightGains.kP),
        // This should return the measurement
        drive::getAverageEncoderDistance,
        // This should return the setpoint (can also be a constant)
        distance,
        // This uses the output
        output -> {
          drive.driveStraight(output, angle);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
