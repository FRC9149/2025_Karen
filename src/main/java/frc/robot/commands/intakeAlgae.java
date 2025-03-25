package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.other.Algae;

public class intakeAlgae extends Command {
    private final Algae subsystem;
    private final boolean intake;

    public intakeAlgae(Algae subsystem, boolean isIntake) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
        intake = isIntake;
    }

    @Override
    public void execute() {
        subsystem.runIntake(intake ? 1 : -1);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.runIntake(0);
    }
}
