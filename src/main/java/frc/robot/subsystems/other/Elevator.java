// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.other;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final SparkMax m1, m2;
  public Elevator() {
    m1 = null;
    m2 = null;
    m1.getEncoder().setPosition(0);
    m2.getEncoder().setPosition(0);
  }

  public void elevatorUp() {
    m1.set(0.1);
    //m2.set(0.1);
  }

  public void elevatorDown() {
    m1.set(-0.1);
    //m2.set(-0.1);
  }

  public void elevatorStop() {
    m1.set(0);
    //m2.set(0);
  }

  public boolean isinverted() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
