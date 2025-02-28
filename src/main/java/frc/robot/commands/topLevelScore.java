package frc.robot.commands;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.other.Elevator;
import frc.robot.subsystems.other.Outake;

public class topLevelScore extends SequentialCommandGroup {

    public topLevelScore(Elevator elevator, Outake outake, int height, double waitTime) {
        addCommands(
            new SequentialCommandGroup(
                new moveElevator(elevator, height),
                new ParallelRaceGroup (
                    new moveElevator(elevator, height+1), 
                    new shootCoral(outake, ShootDirection.CENTER)
                )
            )
        );
    }
}
