// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.other;

import java.util.HashMap;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final HashMap<Integer, Double> heights = new HashMap<Integer, Double>();
  private SparkMax m1, m2;
  private RelativeEncoder e1, e2;
  private PIDController pid = new PIDController(1, 0, 0);
  private SparkMaxConfig firstConfig = new SparkMaxConfig(), secondConfig = new SparkMaxConfig();
  private double encoderTolerance, elevatorHeightMult;


  public Elevator(boolean inverted, double encoderTolerance, double elevatorHeightMult) {
    initMotors(inverted, inverted);
    initHash();

    this.encoderTolerance = encoderTolerance;
    this.elevatorHeightMult = elevatorHeightMult;
  }
  private void initHash() {
    heights.put(0, 0.0);
    heights.put(1, 25.6);
    heights.put(2, 43.0);
    heights.put(3, 66.75);
    heights.put(4, 68.0);
  }
  private void initMotors(boolean inverted1, boolean inverted2) {
    m1 = new SparkMax(1, MotorType.kBrushless);
    m2 = new SparkMax(20, MotorType.kBrushless);

    e1 = m1.getEncoder();
    e2 = m2.getEncoder();

    firstConfig
      .inverted(inverted1)
      .idleMode(IdleMode.kBrake)
    ;
    secondConfig
      .inverted(inverted2)
      .idleMode(IdleMode.kBrake)
    ;

    m1.configure(firstConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m2.configure(secondConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }


  public void setPID(double p, double i, double d) {
    pid = new PIDController(p, i, d);
  }
  public void setPID(PIDController pid) {
    this.pid = pid;
  }
  public void setEncoderTolerance(double tolerance) {
    this.encoderTolerance = tolerance;
  }
  public void setBrake(IdleMode mode) {
    firstConfig.idleMode(mode);
    secondConfig.idleMode(mode);

    m1.configure(firstConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m2.configure(secondConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }


  public void setSpeed(double speed) {
    m1.set(speed);
    m2.set(speed);
  }
  public void elevatorStop() {
    m1.set(0);
    m2.set(0);
  }
  public double getAverageEncoder() {
    return (Math.abs(e1.getPosition()) + Math.abs(e2.getPosition())) / 2;
  }
  public boolean atLevel(int level) {
    return Math.abs(getAverageEncoder() - heights.get(level)) <= encoderTolerance;
  }
  public double getHeight() {
    return getAverageEncoder();
  }
  public double getMaxHeight() {
    return -1;
  }


  /** Moves the elevator to the specified level for the reef.
   * @param level the level to move to 0-3
   */
  public void moveToLevel(int level) {
    // moveToHeight(heights.get(level));
    if (!(level != 0 && getAverageEncoder() <= .7) &&
    !(level == 0 && getAverageEncoder() >= 72)) {
      setSpeed(level == 0 ? .3 : -.3);
    } else {
      setSpeed(0.035);
    }
  }
  /** Moves the elevator to the specified height
   * @param height height in encoder units from the base of the elevator
   */
  public void moveToHeight(double height) {
    setSpeed(pid.calculate(getAverageEncoder(), height));
  }

  @Override
  public void periodic() {
    elevatorHeightMult = Math.abs( (getHeight() / getMaxHeight()) - 1); 
        // (height / maxheight) limit's the values between 0 and 1
        // (x-1) inverts the values so that 1 is now 0 and 0 is now -1
        // Abs just makes sure that the values aren't negative such as inverting 0 into |-1|

    SmartDashboard.putNumber("elevator Encoders", getAverageEncoder());
    SmartDashboard.putNumber("encoder one", e1.getPosition());
    SmartDashboard.putNumber("encoder two", e2.getPosition());
  }
}
