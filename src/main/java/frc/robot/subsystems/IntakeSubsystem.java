// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;

public class IntakeSubsystem extends SubsystemBase {
  // Intake SPARK will be created in the constructor inside a try/catch so we don't fail
  // hard if the REV libraries are not available at runtime.
  private SparkMax intakeMotor = null;
  private GenericEntry intakeAppliedEntry = null;

  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    try {
      intakeMotor = new SparkMax(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);
      intakeMotor.configure(
          Configs.IntakeSubsystem.intakeConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    } catch (Throwable t) {
      // REV library missing or initialization failed — keep intakeMotor null and continue.
      intakeMotor = null;
      System.err.println("IntakeSubsystem: SparkMax unavailable or failed to initialize: " + t.getMessage());
    }

  

      }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    if (intakeMotor != null) {
      intakeMotor.set(power);
    }
  }

  
  

  /**
   * Command to run the intake. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kIntake);
          
        }, () -> {
          this.setIntakePower(0.0);
        }).withName("Intaking");
  }

  /**
   * Command to reverse the intake motor.When the command is interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command runExtakeCommand() {
    return this.startEnd(
        () -> {
          this.setIntakePower(IntakeSetpoints.kExtake);
          
        }, () -> {
          this.setIntakePower(0.0);
                }).withName("Extaking");
  }

  @Override
  public void periodic() {
    // Display subsystem values
    double applied = 0.0;
    if (intakeMotor != null) {
      applied = intakeMotor.getAppliedOutput();
    }
    SmartDashboard.putNumber("Intake | Intake | Applied Output", applied);
    try {
      if (intakeAppliedEntry == null) {
        intakeAppliedEntry = Shuffleboard.getTab("Driver")
            .add("Intake | Intake | Applied Output", intakeMotor.getAppliedOutput())
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(1, 1)
            .getEntry();
      }
      intakeAppliedEntry.setDouble(applied);
    } catch (Throwable ignored) {
    }
  }

}
