package frc.robot.subsystems.other;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase{
    private final SparkMax intakeMotor, armMotor;
    private final boolean intakeMotorReversed, armMotorReversed;

    public Algae(int intakePort, int armPort, boolean intakeReversed, boolean armReversed) {
        intakeMotor = new SparkMax(intakePort, MotorType.kBrushless);
        armMotor = new SparkMax(armPort, MotorType.kBrushless);

        this.intakeMotorReversed = intakeReversed;
        this.armMotorReversed = armReversed;
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }
    public void runArm(double speed) {
        armMotor.set(speed);
    }
}
