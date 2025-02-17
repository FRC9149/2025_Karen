// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robocats.controllers.Ps3;
import frc.robot.commands.moveElevator;
import frc.robot.subsystems.swervedrive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Ps3 m_driverController = new Ps3(0);
  private Pose2d m_pose;

  private final DriveSubsystem swerveBase = new DriveSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    swerveBase.setDefaultCommand(
      new RunCommand(() -> swerveBase.drive(
        m_driverController.getLeftX(),
        m_driverController.getLeftY(),
        m_driverController.getRightX(),
        true
      ), swerveBase)
    );

  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // new Trigger(() -> m_driverController.getBButton())
      // .onTrue(new InstantCommand(swerveBase::zeroGyro));

    // m_driverController.onB().onTrue(new InstantCommand(){
      // @Override
      // public void initialize() {
        // m_pose = swerveBase.getPose();
      // }
    // });
  // 
    // m_driverController.onA().whileTrue(swerveBase.driveTo(m_pose));

    m_driverController.onA().whileTrue(new moveElevator(null, m_driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
