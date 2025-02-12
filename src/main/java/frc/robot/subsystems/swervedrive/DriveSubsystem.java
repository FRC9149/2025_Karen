// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {
    // Robot swerve modules

    private final PIDController m_turnController = new PIDController(1, 0, 0);
    private final double k_turnPidMax = m_turnController.calculate(Math.PI, 0);

    private final SwerveModule m_frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftEncoderPort,
            DriveConstants.kFrontLeftAbsoluteEncoderOffset,
            false,
            false);

    private final SwerveModule m_rearLeft = new SwerveModule(
            DriveConstants.kRearLeftDriveMotorPort,
            DriveConstants.kRearLeftTurningMotorPort,
            DriveConstants.kRearLeftEncoderPort,
            DriveConstants.kRearLeftAbsoluteEncoderOffset,
            false,
            false);

    private final SwerveModule m_frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightEncoderPort,
            DriveConstants.kFrontRightAbsoluteEncoderOffset,
            false,
            false);

    private final SwerveModule m_rearRight = new SwerveModule(
            DriveConstants.kRearRightDriveMotorPort,
            DriveConstants.kRearRightTurningMotorPort,
            DriveConstants.kRearRightEncoderPort,
            DriveConstants.kRearRightAbsoluteEncoderOffset,
            false,
            false);

    // The gyro sensor
    private final AHRS m_gyro = new AHRS(NavXComType.kUSB1);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        m_turnController.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward) (between
     *                      -1 and 1).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     *                      (between -1 and 1)
     * @param rot           Angular rate of the robot. (between -1 and 1)
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        rot *= ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot),
                        DriveConstants.kDrivePeriod));

        setModuleStates(swerveModuleStates);
    }

    public void drive(double xSpeed, double ySpeed, double xHeading, double yHeading) {
        double headingAngle = Math.atan2(yHeading, xHeading); // in radians
        headingAngle += (headingAngle < 0) ? (2 * Math.PI) : 0;

        drive (
            xSpeed,
            ySpeed,
            m_turnController.calculate(m_gyro.getRotation2d().getRadians(), headingAngle) / k_turnPidMax,
            true
        );
    }

    public Command driveTo(Pose2d pose) {
        Pose2d currentPose = getPose();
        DoubleSupplier x = () -> currentPose.getMeasureX().minus(pose.getMeasureX()).in(Meter);
        DoubleSupplier y = () -> currentPose.getMeasureY().minus(pose.getMeasureY()).in(Meter);
        DoubleSupplier r = () -> currentPose.getRotation().minus(pose.getRotation()).getRadians();
        // TODDO fix driveTo Command
        class DriveTo extends Command {
            DriveSubsystem m_subsystem;
            DoubleSupplier x, y, r;

            public DriveTo(DriveSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
                m_subsystem = subsystem;
                this.x = x;
                this.y = y;
                this.r = r;
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                m_subsystem.drive(x.getAsDouble(), y.getAsDouble(), r.getAsDouble(), true);
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public void end(boolean interrupted) {
                if (!interrupted) {
                    m_subsystem.drive(0, 0, 0, false);
                }
            }
        }

        return new DriveTo(this, x, y, r);
        // TODO fix driveTo
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
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
