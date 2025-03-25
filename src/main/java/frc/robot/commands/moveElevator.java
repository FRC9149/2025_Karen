package frc.robot.commands;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.other.Elevator;

/**
 * <p>Moves the elevator to one of the specified levels.</p><br>
 * <p>-1 for all the way down </p>
 */
public class moveElevator extends Command {
    private int height = 0;
    private final Elevator subsystem;
    
    //called once when object is created
    // does not run on button press only when the robot starts
    public moveElevator(Elevator subsystem, int level) {
        height = level;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    // called over and over again
    // like a while (true) loop
    @Override
    public void execute() {
        subsystem.moveToLevel(height);
    }  

    // called once after isFinished returns true
    // interrupted is true when another command uses the same subsystem
    @Override
    public void end(boolean interrupted) {
        subsystem.elevatorStop(height != -1); // if height isn't at the bottom. use braking
    }

    // returns true when the command should end
    @Override
    public boolean isFinished() {
        // return false;
        return subsystem.atLevel(height);
    }
}
