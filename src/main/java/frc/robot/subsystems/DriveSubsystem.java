package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  
  private final MotorControllerGroup m_leftMotors =
  new MotorControllerGroup(
    new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless),
    new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless),
    new CANSparkMax(DriveConstants.kLeftMotor3Port, MotorType.kBrushless));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(
    new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless),
    new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless),
    new CANSparkMax(DriveConstants.kRightMotor3Port, MotorType.kBrushless));


  
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Define gyro
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */

  // Local copies of arcade drive speed and rotation. So we can do a rotation adjustment while moving
  // (fadeaway shot)
  private double xSpeed = 0;
  private double xRotation = 0;
 
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  // Should we keep this around, I think so
  public void arcadeDrive(double fwd, double rot) {
    xSpeed = fwd;
    xRotation = rot;
  }

  public void arcadeDriveSpeed(double fwd) {
    xSpeed = fwd;
  }

  public void arcadeDriveRotation(double rot) {
    xRotation = rot;
  }

  public void periodic() {
    // Set the drive speeds, but don't square them. Let the caller do that
    m_drive.arcadeDrive(xSpeed, xRotation, false);
    
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("xRotation", xRotation);
  }
  
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
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
