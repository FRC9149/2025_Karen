package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.other.Outake;
public class shootCoral extends Command {
    private final ShootDirection direction;
    private final Outake subsystem;
    private boolean wasTripped = false;
    private double countdown = 10;
    
    public shootCoral(Outake subsystem, ShootDirection direction) {
        this.direction = direction;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.run(1, direction);
    }
    @Override
    public void execute() {
        if(subsystem.getSecond() || wasTripped) {
            subsystem.run(1, direction);
            wasTripped = true;
            countdown -= 0.1;
        }
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        wasTripped = false;
    }

    @Override
    public boolean isFinished() {
        return countdown <= 0;
    }
}
