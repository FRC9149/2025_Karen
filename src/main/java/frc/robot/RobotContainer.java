// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.robocats.controllers.DDR;
import com.robocats.controllers.Ps3;
import com.robocats.controllers.RevGamePad;
import com.robocats.controllers.Streetfightercontrol;

import frc.robot.commands.ShootDirection;
import frc.robot.commands.moveElevator;
import frc.robot.commands.shootCoral;
import frc.robot.commands.topLevelScore;
import frc.robot.subsystems.other.Elevator;
import frc.robot.subsystems.other.Outake;
import frc.robot.subsystems.swervedrive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final RevGamePad m_driverController = new RevGamePad(0);
  private final Streetfightercontrol m_operatorController = new Streetfightercontrol(1);
  private double elevatorHeightMult = 0;
  private Pose2d m_pose;

  private final DriveSubsystem swerveBase = new DriveSubsystem();
  private final Outake outake = new Outake();
  private final Elevator elevator = new Elevator(true, 0.2, elevatorHeightMult);
  // private final Elevator elevator = new Elevator(false, 0.1, elevatorHeightMult);
  // private final Outake outake = new Outake();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // NamedCommands.registerCommand("shoot", new shootCoral(outake, ShootDirection.LEFT));

    // Configure the trigger bindings
    configureBindings();

    swerveBase.setDefaultCommand(
      new RunCommand(() -> swerveBase.drive(
        m_driverController.getLeftY(),
        m_driverController.getLeftX(),
        -m_driverController.getRightX(),
        m_driverController.getRightY()
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

    // m_operatorController.onLeftStickIn().whileTrue(new moveElevator(elevator, 0));
    // m_operatorController.onA().whileTrue(new moveElevator(elevator, 1));
    // m_operatorController.onX().whileTrue(new moveElevator(elevator, 2));
    // m_operatorController.onLeftBumper().whileTrue(new moveElevator(elevator, 3));

    // m_driverController.onRightBumper().onTrue(new RunCommand(()->  // When Start is pressed
    //   swerveBase.setDefaultCommand(                             // set the driving mode
    //     new RunCommand(()-> swerveBase.drive(                   // to use right x as linear velocity not as a rotation target
    //       m_driverController.getLeftX(), //* elevatorHeightMult,
    //       m_driverController.getLeftY(), //* elevatorHeightMult,
    //       m_driverController.getRightX(), 
    //       true
    //     ), swerveBase)
    //   )
    // , swerveBase));

    // m_driverController.onLeftBumper().onTrue( new RunCommand(()->
    //   swerveBase.setDefaultCommand( new RunCommand(()-> swerveBase.drive(
    //     m_driverController.getLeftX(), //* elevatorHeightMult,
    //     m_driverController.getLeftY(), //* elevatorHeightMult, 
    //     m_driverController.getRightX(),
    //     m_driverController.getRightY()
    //   ), swerveBase)
    // ), swerveBase));

    // m_driverController.onRightTrigger(.5).whileTrue(new shootCoral(outake, ShootDirection.CENTER));
    // m_driverController.onRightBumper()             .whileTrue(new shootCoral(outake, ShootDirection.RIGHT ));



    /*
    m_driverController.onStart().onTrue(new Command(){
      @Override
      public void initialize() {
        // swerveBase.resetGyro();
        System.out.println("Reset Gyro");
      }
    });*/

    //m_driverController.onRightTrigger(.5).whileTrue(new shootCoral(outake, ShootDirection.CENTER));
    /*
       m_driverController.onRightBumper().whileTrue(new moveElevator(elevator, 0));
    m_driverController.onLeftBumper().whileTrue(new moveElevator(elevator, 1));
    m_driverController.onDPadLeft().whileTrue(new shootCoral(outake, ShootDirection.LEFT));
    m_driverController.onDPadRight().whileTrue(new shootCoral(outake, ShootDirection.RIGHT));
    m_driverController.onDPadDown().whileTrue(new topLevelScore(elevator, outake, 0, 2));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
    // return new PathPlannerAuto("First");
  }
}
