// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.robocats.controllers.RevGamePad;
import com.robocats.controllers.Streetfightercontrol;
import com.robocats.swerve.SwerveConfig;
import com.robocats.swerve.SwerveSubsystem;
import com.robocats.swerve.gyroscope.ahrsGyro;
import com.robocats.vision.AprilCamera;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ShootDirection;
import frc.robot.commands.intakeAlgae;
import frc.robot.commands.moveAlgaeArm;
import frc.robot.commands.moveElevator;
import frc.robot.commands.rawMoveElevator;
import frc.robot.commands.shootCoral;
import frc.robot.commands.topLevelScore;
import frc.robot.subsystems.other.Algae;
import frc.robot.subsystems.other.Elevator;
import frc.robot.subsystems.other.Outake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // initalize all the controllers
  private final RevGamePad m_driverController = new RevGamePad(0);
  private final Streetfightercontrol m_operatorController = new Streetfightercontrol(1);

  // initalize all the subsystems like the drivetrain and elevator
  private final SwerveSubsystem swerveBase = new SwerveSubsystem(new SwerveConfig(
    4, 3 * Math.PI, .1016, TimedRobot.kDefaultPeriod, 
    DriveConstants.kDriveKinematics, 
    new com.robocats.swerve.ModuleConfig(
      4,8,2,11,
      7,9,6,13,
      15,17,14,16,
      .603516,.603516,.713623,.458008,
      false, true, true, false), 
    new ahrsGyro(NavXComType.kUSB1, Math.PI/2, false),
    true
  ), new PIDController(0,0,0));
  private final Outake outake = new Outake();
  private final Elevator elevator = new Elevator(true, 0.15);
  private final AprilCamera camera = new AprilCamera("April_Camera", new Transform3d(.22, .28, .51, new Rotation3d(0, 0, 7.5 * Math.PI / 180)));
  private final Algae algae = new Algae(17, 5, true, true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //setsup all the commands for autons (I spelt that wrong but idc)
    NamedCommands.registerCommand("shoot", new shootCoral(outake, ShootDirection.CENTER));
    NamedCommands.registerCommand("shoot l0", new shootCoral(outake, ShootDirection.RIGHT));
    NamedCommands.registerCommand("shoot l3", new topLevelScore(elevator, outake));
    NamedCommands.registerCommand("base", new moveElevator(elevator, -1));
    NamedCommands.registerCommand("l0", new moveElevator(elevator, 0));
    NamedCommands.registerCommand("l1", new moveElevator(elevator, 1));
    NamedCommands.registerCommand("l2", new moveElevator(elevator, 2));
    NamedCommands.registerCommand("l3", new moveElevator(elevator, 3));

    configureBindings();

    // Configures the buttons to drive the robot
    // Runs when no other command uses the swervedrive
    swerveBase.setDefaultCommand(
      new RunCommand(() -> swerveBase.drive(
        m_driverController.getLeftY(),
        m_driverController.getLeftX(),
        m_driverController.getRightX(),
        m_driverController.getRightY()
      ), swerveBase)
    );
    swerveBase.setupPathPlanner();
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
    //zeros the gyroscope, changing which way is forwards.
    m_driverController.onX().onTrue(new InstantCommand(()->swerveBase.swerveConfig.gyroscope().zero(), swerveBase));
    
    //aligns the robot with tag
    m_driverController.onA().whileTrue(
      new RunCommand( () -> {
        camera.update();
        var pid = new PIDController(0.00075, 1.5, .00);
        var drivePid = new PIDController(1, 1, 1);
        swerveBase.drive(
          -drivePid.calculate(camera.getDistanceY(), .26),
          -drivePid.calculate(camera.getDistanceX(), 0),
          pid.calculate(camera.getZAngleRadians(), 0),
        false);
      },
      swerveBase)
    );

    m_driverController.onY().whileTrue(
      new RunCommand( () -> {
        camera.update();
        var pid = new PIDController(0.00075, 1.5, 0);
        var drivepid = new PIDController(1, 1, 1);
        swerveBase.drive(
          -drivepid.calculate(camera.getDistanceY(), -.52),
          -drivepid.calculate(camera.getDistanceX(), 0),
          pid.calculate(camera.getZAngleRadians(), 0),
          false
        );
      },
      swerveBase)
    );

    //shoots the coral
    m_driverController.onRightTrigger(.5).whileTrue(new shootCoral(outake, ShootDirection.CENTER));

    // Code to move the elevator with the side panel
    m_operatorController.onLeftStickIn().whileTrue(new moveElevator(elevator, 0));
    m_operatorController.onA().whileTrue(new moveElevator(elevator, 1));
    m_operatorController.onX().whileTrue(new moveElevator(elevator, 2));
    m_operatorController.onLeftBumper().whileTrue(new moveElevator(elevator, 3));
    m_operatorController.onRightBumper().whileTrue(new topLevelScore(elevator, outake));

    // incase operator controller disconnects, use this
    m_driverController.onB().and(m_driverController.onDPadDown()).whileTrue(new moveElevator(elevator, 0));
    m_driverController.onB().and(m_driverController.onDPadLeft()).whileTrue(new moveElevator(elevator, 1));
    m_driverController.onB().and(m_driverController.onDPadRight()).whileTrue(new moveElevator(elevator, 2));
    m_driverController.onB().and(m_driverController.onDPadUp()).whileTrue(new moveElevator(elevator, 3));



    // moves the elevator up or down with the controller
    // Right bumprt up
    // Left bumper down
    m_driverController.onRightBumper().whileTrue(new rawMoveElevator(elevator, true));
    m_driverController.onLeftBumper().whileTrue(new moveElevator(elevator, -1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }
}