// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that controls an intake lift motor. Provides simple open-loop commands
 * to move the lift up or down and to stop it.
 */
public class IntakeLiftSubsystem extends SubsystemBase {
  // Use constants (mirrors IntakeSubsystem patterns)
  private final SparkMax liftMotor =
      new SparkMax(frc.robot.Constants.IntakeLiftSubsystemConstants.kIntakeLiftMotorCanId, MotorType.kBrushless);

  public IntakeLiftSubsystem() {
    // Optionally apply safe/persistent config if available in Configs; keep default for now
    // Apply the same intake config used by the intake subsystem so behavior matches
    try {
      liftMotor.configure(
          frc.robot.Configs.IntakeSubsystem.intakeConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    } catch (Throwable ignored) {
      // If Configs is unavailable in this environment, ignore — defaults will be used at runtime.
    }
  }

  /** Set the lift motor power in range [-1, 1]. Positive should move the lift up. */
  public void setLiftPower(double power) {
    liftMotor.set(power);
  }

  /** Stop the lift motor. */
  public void stopLift() {
    liftMotor.set(0.0);
  }

  /** Command to run the lift upward while the command is active. */
  public Command runLiftUpCommand() {
    return this.startEnd(
        () -> setLiftPower(frc.robot.Constants.IntakeLiftSubsystemConstants.LiftSetpoints.kLiftUp),
        this::stopLift)
        .withName("Lift Up");
  }

  /** Command to run the lift downward while the command is active. */
  public Command runLiftDownCommand() {
    return this.startEnd(
        () -> setLiftPower(frc.robot.Constants.IntakeLiftSubsystemConstants.LiftSetpoints.kLiftDown),
        this::stopLift)
        .withName("Lift Down");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeLift | Applied Output", liftMotor.getAppliedOutput());
    try {
      // Mirror IntakeSubsystem behaviour: add a Shuffleboard text view if available
      edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Driver")
          .add("IntakeLift | Applied Output", liftMotor.getAppliedOutput())
          .withWidget(edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.kTextView)
          .withSize(1, 1);
    } catch (Throwable ignored) {
    }
  }
}
