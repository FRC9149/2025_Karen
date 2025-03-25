package frc.robot.commands;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.other.Elevator;
import frc.robot.subsystems.other.Outake;

public class topLevelScore extends SequentialCommandGroup {

    public topLevelScore(Elevator elevator, Outake outake) {
        addCommands(
            new ParallelCommandGroup (
                new moveElevator(elevator, 4), 
                new shootCoral(outake, ShootDirection.CENTER)
            )
        );
    }
}
