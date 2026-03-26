// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that controls a lift motor for the intake. Provides simple open-loop commands
 * to move the lift up or down and to stop it.
 */
public class LiftSubsystem extends SubsystemBase {
  // Use constants (mirrors IntakeSubsystem patterns)
  private SparkMax liftMotor = null;

  public LiftSubsystem() {
    // Try to create and configure the SPARK MAX; if the REV libraries aren't
    // available at runtime this will fail without bringing down the robot code.
    try {
      liftMotor = new SparkMax(frc.robot.Constants.LiftSubsystemConstants.kLiftMotorCanId,
          MotorType.kBrushless);
      // Configuration object referenced previously (frc.robot.Configs.LiftSubsystem.liftConfig)
      // does not exist in this project; skip calling configure here to avoid a compile error.
      // If you have a valid SparkMax configuration object, call liftMotor.configure(...) with it.
    } catch (Throwable t) {
      liftMotor = null;
      System.err.println("LiftSubsystem: SparkMax unavailable or failed to initialize: " + t.getMessage());
    }
  }

  /** Set the lift motor power in range [-1, 1]. Positive should move the lift up. */
  public void setLiftPower(double power) {
    if (liftMotor != null) {
      liftMotor.set(power);
    }
  }

  /** Stop the lift motor. */
  public void stopLift() {
    if (liftMotor != null) {
      liftMotor.set(0.0);
    }
  }

  public Command runLiftUpCommand() {
    return this.startEnd(
        () -> setLiftPower(frc.robot.Constants.LiftSubsystemConstants.LiftSetpoints.kLiftUp),
        this::stopLift)
        .withName("Lift Up");
  }

  /** Command to run the lift downward while the command is active. */
  public Command runLiftDownCommand() {
    return this.startEnd(
        () -> setLiftPower(frc.robot.Constants.LiftSubsystemConstants.LiftSetpoints.kLiftDown),
        this::stopLift)
        .withName("Lift Down");
  }

  @Override
  public void periodic() {
    double applied = 0.0;
    if (liftMotor != null) {
      applied = liftMotor.getAppliedOutput();
    }
    SmartDashboard.putNumber("Lift | Applied Output", applied);
    try {
      // Mirror IntakeSubsystem behaviour: add a Shuffleboard text view if available
      edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Driver")
          .add("Lift | Applied Output", applied)
          .withWidget(edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.kTextView)
          .withSize(1, 1);
    } catch (Throwable ignored) {
    }
  }
}
