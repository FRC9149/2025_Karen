package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.other.Algae;

public class moveAlgaeArm extends Command {
    private final Algae subsystem;
    private final boolean down;

    public moveAlgaeArm(Algae subsystem, boolean isDown) {
        this.subsystem = subsystem;
        down = isDown;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.runArm(down ? .25 : -.25);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.runArm(0);
    }
}
