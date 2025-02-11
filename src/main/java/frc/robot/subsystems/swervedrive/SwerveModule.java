// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  
  private final RelativeEncoder m_driveEncoder;

  private final CANcoder m_absolute_encoder;

  private final double m_absoluteOffset;
  private final boolean m_absoluteReversed, m_motorReversed;

  /**
   * @return The distance that the module has driven since reset (meters)
   */
  private double getDriveDistance() {
    // if(m_driveMotor.getDeviceId() == 2) System.out.println(m_driveEncoder.getPosition());

    return m_driveEncoder.getPosition() * (Math.PI * Math.pow(ModuleConstants.kWheelDiameterMeters, 2)) ;
    // multiple the rotation amount by the circumfrence to get the distance traveled [PI(r^2)]
  }

  /**
   * @return The current angle of the module (radians)
   */
  private double getTurnDistance() {
    // if(m_absolute_encoder.getDeviceID() == 15) System.out.println(m_absolute_encoder.getAbsolutePosition().getValueAsDouble());

    double encoderValue = m_absolute_encoder.getAbsolutePosition().getValueAsDouble() + (m_absoluteReversed ? -1 : 1) * m_absoluteOffset;

    encoderValue += m_motorReversed ? .5 : 0;
    while (encoderValue < 0) encoderValue += 1;
    while (encoderValue > 1) encoderValue -= 1; // 1 = 360 degrees so we're finding a coterminal angle that that is between 0 and 1 rotations
    //probably could skip this step but it's not a big deal

    return encoderValue * 2 * Math.PI; //convert to radians
  }

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param encoderChannel      The channel of the absolute encoder
   * @param encoderOffset       The offset of the absolute encoder
   * @param absoluteEncoderReversed Whether or not the absolute encoder is reversed
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int encoderChannel,
      double encoderOffset,
      boolean motorReversed,
      boolean absoluteEncoderReversed) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_absolute_encoder = new CANcoder(encoderChannel);

    m_absoluteOffset = encoderOffset;
    m_motorReversed = motorReversed;
    m_absoluteReversed = absoluteEncoderReversed;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(),
        new Rotation2d(getTurnDistance()));
    // original

    // return new SwerveModuleState(
    // m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(),
        new Rotation2d(getTurnDistance()));
    // original

    // return new SwerveModulePosition(
    // m_driveEncoder.getDistance(), new
    // Rotation2d(m_turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getTurnDistance());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
        desiredState.speedMetersPerSecond);
    // m_drivePIDController.calculate(m_driveEncoder.getRate(),
    // desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurnDistance(), desiredState.angle.getRadians());
    // m_turningPIDController.calculate(m_turningEncoder.getDistance(),
    // desiredState.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }
}
