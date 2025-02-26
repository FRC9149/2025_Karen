// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {
    // Robot swerve modules

    private final PIDController m_turnController = new PIDController(0.5, 0.0, 0.0);
    private RobotConfig config;

    private final SwerveModule m_frontLeft = new SwerveModule(
        "fl",
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftEncoderPort,
            DriveConstants.kFrontLeftAbsoluteEncoderOffset,
            false);

    private final SwerveModule m_rearLeft = new SwerveModule(
        "bl",
            DriveConstants.kRearLeftDriveMotorPort,
            DriveConstants.kRearLeftTurningMotorPort,
            DriveConstants.kRearLeftEncoderPort,
            DriveConstants.kRearLeftAbsoluteEncoderOffset,
            true);

    private final SwerveModule m_frontRight = new SwerveModule(
        "fr",
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightEncoderPort,
            DriveConstants.kFrontRightAbsoluteEncoderOffset,
            true);

    private final SwerveModule m_rearRight = new SwerveModule(
        "br",
            DriveConstants.kRearRightDriveMotorPort,
            DriveConstants.kRearRightTurningMotorPort,
            DriveConstants.kRearRightEncoderPort,
            DriveConstants.kRearRightAbsoluteEncoderOffset,
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
/*  Don't ask...
His name is Jeremy...




                    Silent streams                        silent streams
                and whispering winds                   and whispering winds
              shadows dance beneath the moon       shadows dance beneath the moon
             echoes of the nightingale's song     echoes of the nightingale's song
              stars align in cosmic harmony        stars align in cosmic harmony
                   dreams awaken softly                dreams awaken softly



                   
              beneath the twilight                          beneath the twilight
       mysteries unfold in the night's embrace        mysteries unfold in the night's embrace
              secrets whispered among the trees secrets whispered among the trees
                    the forest breathes deeply the forest breathes deeply
                            ------------------------------------
                                A poem crafted from Chat GPT
                                    visuals by humans




(P.S. He says hello.)
*/
    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        m_turnController.enableContinuousInput(0, 2 * Math.PI);

        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        SmartDashboard.putNumber("gyro", m_gyro.getRotation2d().getRadians());
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
        
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

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        });
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

        // while (headingAngle > 2 * Math.PI) headingAngle -= 2 * Math.PI;
        // while (headingAngle < 0) headingAngle += 2 * Math.PI; // convert from -pi->pi to 0->2pi
        drive (
            xSpeed,
            ySpeed,
            xHeading == 0 && yHeading == 0 ? 0 : 
                m_turnController.calculate(m_gyro.getRotation2d().getRadians(), headingAngle),
            true
        );
    }
    // public void drive(ChassisSpeeds speeds) {
        // setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    // }

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

    public void resetGyro() {
        m_gyro.reset();
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

    /**
   * Setup AutoBuilder for PathPlanner.
   */
//   public void setupPathPlanner() {
    // AutoBuilder.configure(
            // this::getPose, // Robot pose supplier
            // this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            // this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            // (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            // new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    // new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    // new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            // ),
            // config, // The robot configuration
            // () -> {
            //   Boolean supplier that controls when the path will be mirrored for the red alliance
            //   This will flip the path being followed to the red side of the field.
            //   THE ORIGIN WILL REMAIN ON THE BLUE SIDE
// 
            //   var alliance = DriverStation.getAlliance();
            //   if (alliance.isPresent()) {
                // return alliance.get() == DriverStation.Alliance.Red;
            //   }
            //   return false;
            // },
            // this // Reference to this subsystem to set requirements
    // );
//   }

}
