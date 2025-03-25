package frc.robot.subsystems.other;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.robocats.sensors.distanceSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootDirection;

public class Outake extends SubsystemBase{
    private final SparkMax mLeft, mRight;
    private final distanceSensor s2;

    public Outake() {
        mLeft = new SparkMax(10, MotorType.kBrushless);
        mRight = new SparkMax(3, MotorType.kBrushless);

        s2 = new distanceSensor(1);
    }

    public void run(double speed, ShootDirection direction) {
        if(direction == ShootDirection.INTAKE && getSensor()) {
            mLeft.set(0);
            mRight.set(0);
            return;
        }
        mLeft.set ( -speed/ (direction == ShootDirection.LEFT ? 2 : 1));
        mRight.set( speed/ (direction == ShootDirection.RIGHT ? 2 : 1));
    }
    public void stop() {
        mLeft.set(0);
        mRight.set(0);
    }
    public boolean getSensor() {
        return s2.isTripped();
    }
}
