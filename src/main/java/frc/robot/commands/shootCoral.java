package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.other.Outake;
public class shootCoral extends Command {
    private final ShootDirection direction;
    private final Outake subsystem;
    
    public shootCoral(Outake subsystem, ShootDirection direction) {
        this.direction = direction;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void execute() {
        subsystem.run(1, direction);
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
