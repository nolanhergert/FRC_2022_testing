// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class TurnToAngleProfiled extends ProfiledPIDCommand {

  private static final SimpleMotorFeedforward m_feedforward_turning = 
  new SimpleMotorFeedforward(Constants.DriveConstants.kStaticVoltsTurning, Constants.DriveConstants.kVelocityVoltsTurning);

  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngleProfiled(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        new ProfiledPIDController(
            Constants.DriveConstants.kTurningP,
            Constants.DriveConstants.kTurningI,
            Constants.DriveConstants.kTurningD,
            new TrapezoidProfile.Constraints(
              Constants.DriveConstants.kMaxTurnRateDegPerS,
              Constants.DriveConstants.kMaxTurnAccelerationDegPerSSquared)),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Turn robot with feedforward in the loop too
        (output, setpoint) -> drive.arcadeDriveRotation(output + m_feedforward_turning.calculate(setpoint.velocity)),
        // Require the drive
        drive);


    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kToleranceDeg, DriveConstants.kRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}