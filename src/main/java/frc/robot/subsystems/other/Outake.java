package frc.robot.subsystems.other;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.robocats.sensors.distanceSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootDirection;

public class Outake extends SubsystemBase{
    private final SparkMax mLeft, mRight;
    private final distanceSensor s1, s2;

    public Outake() {
        mLeft = new SparkMax(10, MotorType.kBrushless);
        mRight = new SparkMax(3, MotorType.kBrushless);

        s1 = new distanceSensor(0);
        s2 = new distanceSensor(1);
    }

    public void run(double speed, ShootDirection direction) {
        mLeft.set ( -speed);
        mRight.set( speed);
    }
    public void stop() {
        mLeft.set(0);
        mRight.set(0);
    }

    public boolean getfirst() {
        return s1.isTripped();
    }
    public boolean getSecond() {
        return s2.isTripped();
    }
}
