// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants;

/**
 * Feeder subsystem that controls the feeder motor independently from the flywheel.
 */
public class FeederSubsystem extends SubsystemBase {

  private SparkMax feederMotor =
      new SparkMax(ShooterSubsystemConstants.kFeederMotorCanId, MotorType.kBrushless);


  

  /** Set the feeder motor power in the range [-1, 1]. */
  public void setFeederPower(double power) {
    feederMotor.set(power);
  }

  /**
   * Command to run only the feeder. Stops the feeder when interrupted.
   */
  public Command runFeederCommand() {
    return this.startEnd(() -> setFeederPower(FeederSetpoints.kFeed), () -> setFeederPower(0.0)).withName("Feeding");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder | Applied Output", feederMotor.getAppliedOutput());
    SmartDashboard.putNumber("Feeder | Current", feederMotor.getOutputCurrent());
  }
}
