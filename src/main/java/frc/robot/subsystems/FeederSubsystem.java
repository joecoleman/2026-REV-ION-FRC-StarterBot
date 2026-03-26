// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import frc.robot.Constants.ShooterSubsystemConstants;

/**
 * Feeder subsystem that controls the feeder motor independently from the flywheel.
 */
public class FeederSubsystem extends SubsystemBase {

  private SparkMax feederMotor = null;
  private GenericEntry feederAppliedEntry = null;
  private GenericEntry feederCurrentEntry = null;

  /** Creates a new FeederSubsystem. Initializes the SparkMax if available. */
  public FeederSubsystem() {
    try {
      feederMotor = new SparkMax(ShooterSubsystemConstants.kFeederMotorCanId, MotorType.kBrushless);
    } catch (Throwable t) {
      feederMotor = null;
      System.err.println("FeederSubsystem: SparkMax unavailable or failed to initialize: " + t.getMessage());
    }
  }


  

  /** Set the feeder motor power in the range [-1, 1]. */
  public void setFeederPower(double power) {
    if (feederMotor != null) {
      feederMotor.set(power);
    }
  }

  /**
   * Command to run only the feeder. Stops the feeder when interrupted.
   */
  public Command runFeederCommand() {
    return this.startEnd(() -> setFeederPower(FeederSetpoints.kFeed), () -> setFeederPower(0.0)).withName("Feeding");
  }

  public void setFeeder(boolean enabled) {
    if (enabled) {
      setFeederPower(ShooterSubsystemConstants.FeederSetpoints.kFeed);
    } else {
      setFeederPower(0.0);
    }
  }

  @Override
  public void periodic() {
    double applied = 0.0;
    double current = 0.0;
    if (feederMotor != null) {
      applied = feederMotor.getAppliedOutput();
      current = feederMotor.getOutputCurrent();
    }
    SmartDashboard.putNumber("Feeder | Applied Output", applied);
    SmartDashboard.putNumber("Feeder | Current", current);
    try {
      if (feederAppliedEntry == null) {
        feederAppliedEntry = Shuffleboard.getTab("Driver")
            .add("Feeder | Applied Output", applied)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(1, 1)
            .getEntry();
      }
      feederAppliedEntry.setDouble(applied);

      if (feederCurrentEntry == null) {
        feederCurrentEntry = Shuffleboard.getTab("Driver")
            .add("Feeder | Current", current)
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(1, 1)
            .getEntry();
      }
      feederCurrentEntry.setDouble(current);
    } catch (Throwable ignored) {
      // If Shuffleboard not available, ignore.
    }
  }
}
