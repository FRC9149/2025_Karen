// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package main.java.frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Example of a TankDrive subsystem
 *  Uses Canspark and an xbox controller
 *  Sorry if it doesn't work, I had to write this without autocomplete for some reason
 */
public class TankDriveTrain extends SubsystemBase {
  private XboxController controller;
  private CANSparkMax leftMotor  = new CANSparkMax(DrivetrainConstants.kLeftDriveMotorPort, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(DrivetrainConstants.kRightDriveMotorPort, MotorType.kBrushless); 

  /**
   * @param controller The xbox controller used to move the robot
   * @param isMotorReversed a pair of bools that describe if the motors should be reversed. The first value (key) should be the left motor and the second (value) should be the right motor
   */
  public TankDriveTrain(XboxController controller, Pair<Bool, Bool> isMotorReversed) {
    controller = controller;
    leftMotor.setInverted(isMotorReversed.getKey());
    leftMotor.setInverted(isMotorReversed.getValue());
  }

  public void drive() {
    leftMotor.set(controller.getLeftY());
    rightMotor.set(controller.getRightY());
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @Override public void periodic() {}
  @Override public void simulationPeriodic() {}
}
