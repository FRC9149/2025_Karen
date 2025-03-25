package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.other.Elevator;

/**
 * Moves the elevator up or down. Doesn't go to a certain height
 */
public class rawMoveElevator extends Command {
    Elevator subsystem;
    boolean isUp;
    public rawMoveElevator(Elevator subsystem, boolean isUp) {
        this.subsystem = subsystem;
        this.isUp = isUp;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setSpeed(isUp ? .3 : -.1);
    }

    @Override 
    public void end(boolean interrupted) {
        subsystem.elevatorStop(true);
    }

    @Override
    public boolean isFinished() {
        return subsystem.getAverageEncoder() >= 70;
    }
}
