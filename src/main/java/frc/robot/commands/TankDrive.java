// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package main.java.frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import main.java.frc.robot.subsystems.TankDriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

public class TankDrive extends Command {
  private TankDrive subsystem;
  /**
   * @param m_subsystem The subsystem used by this command.
   */
  public TankDrive(TankDriveTrain m_subsystem) {
    subsystem = m_subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.drive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
