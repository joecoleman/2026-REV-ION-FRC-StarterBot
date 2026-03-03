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
import frc.robot.Configs;
import frc.robot.Constants.ShooterSubsystemConstants;

/**
 * Simplified shooter subsystem that uses open-loop control for the flywheel.
 * The feeder has been moved to its own subsystem.
 */
public class ShooterSubsystem extends SubsystemBase {

  private SparkMax flywheelMotor =
      new SparkMax(ShooterSubsystemConstants.kFlywheelMotorCanId, MotorType.kBrushless);
  private SparkMax flywheelFollowerMotor =
      new SparkMax(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId, MotorType.kBrushless);

  // Open-loop power target for the flywheel in the range [-1, 1]
  private double flywheelTargetPower = 0.0;
  private static final double kDefaultFlywheelPower = 1.0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Configure the SPARKs (keeps previous config behavior)
    flywheelMotor.configure(
        Configs.ShooterSubsystem.flywheelConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    flywheelFollowerMotor.configure(
        Configs.ShooterSubsystem.flywheelFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    
  }

  /** Set the flywheel power (open-loop). */
  private void setFlywheelPower(double power) {
    flywheelMotor.set(power);
    // Mirror the leader on the follower motor
    flywheelFollowerMotor.set(power);
    flywheelTargetPower = power;
  }

  /**
   * Command to run the flywheel motors open-loop. When interrupted the motors stop.
   */
  public Command runFlywheelCommand() {
    return this.startEnd(
        () -> this.setFlywheelPower(kDefaultFlywheelPower),
        () -> this.setFlywheelPower(0.0)).withName("Spinning Up Flywheel");
  }

  /**
   * Alias for running the flywheel — kept for API compatibility.
   */
  public Command runShooterCommand() {
    return runFlywheelCommand().withName("Shooting");
  }

  @Override
  public void periodic() {
    // Display subsystem values (feeder moved to FeederSubsystem)
    SmartDashboard.putNumber("Shooter | Flywheel | Applied Output", flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel Follower | Applied Output", flywheelFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter | Flywheel | Target Power", flywheelTargetPower);
  }

}
