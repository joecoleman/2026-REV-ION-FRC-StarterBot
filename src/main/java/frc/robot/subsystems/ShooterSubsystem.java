// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;

public class ShooterSubsystem extends SubsystemBase {

  // VictorSPX motors for open-loop brushed flywheel control
  private final WPI_VictorSPX flywheelMotor =
      new WPI_VictorSPX(ShooterSubsystemConstants.kFlywheelMotorCanId);

  private final WPI_VictorSPX flywheelFollowerMotor =
      new WPI_VictorSPX(ShooterSubsystemConstants.kFlywheelFollowerMotorCanId);

  // Track last commanded percent output
  private double flywheelTargetOutput = 0.0;
  // Track whether shooter is enabled (open-loop on)
  private boolean shooterEnabled = false;

  public ShooterSubsystem() {
    // Configure VictorSPX for open-loop operation. These calls require CTRE Phoenix on the classpath.
    try {
      flywheelMotor.configFactoryDefault();
      flywheelMotor.setInverted(false);
      flywheelMotor.setNeutralMode(NeutralMode.Coast);

      flywheelFollowerMotor.configFactoryDefault();
      flywheelFollowerMotor.setInverted(true);
      flywheelFollowerMotor.setNeutralMode(NeutralMode.Coast);
      flywheelFollowerMotor.follow(flywheelMotor);
    } catch (Throwable t) {
      // If CTRE isn't available in the editor, ignore — it will work at build time when the
      // Phoenix dependency is resolved.
    }

    System.out.println("---> ShooterSubsystem initialized");
  }

  // Trigger: Is the flywheel spinning (open-loop, based on commanded/actual percent)?
  public final Trigger isFlywheelSpinning =
      new Trigger(() -> Math.abs(flywheelMotor.get()) > FlywheelSetpoints.kSpinThresholdPercent);

  public final Trigger isFlywheelSpinningBackwards =
      new Trigger(() -> flywheelMotor.get() < -FlywheelSetpoints.kSpinThresholdPercent);

  public final Trigger isFlywheelStopped = new Trigger(() -> Math.abs(flywheelMotor.get()) < 0.01);

  /** Open-loop control: set percent output for flywheel (range -1.0..1.0) */
  public void setShooterOutput(double percent) {
    flywheelMotor.set(percent);
    flywheelTargetOutput = percent;
    shooterEnabled = Math.abs(percent) > 1e-6;
  }

  /** Turn the shooter on full (uses FlywheelSetpoints.kShootPercent) */
  public void setShooterOn() {
    setShooterOutput(FlywheelSetpoints.kShootPercent);
    // Ensure follower mirrors leader
    try {
      flywheelFollowerMotor.follow(flywheelMotor);
    } catch (Throwable ignored) {
    }
  }

  /** Turn the shooter off */
  public void setShooterOff() {
    setShooterOutput(0.0);
  }

  /** Toggle shooter on/off (uses kShootPercent when enabling) */
  public void toggleShooter() {
    if (shooterEnabled) {
      setShooterOff();
    } else {
      setShooterOn();
    }
  }

  /** Returns true if shooter is currently enabled (open-loop output non-zero) */
  public boolean isShooterEnabled() {
    return shooterEnabled;
  }

  /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runFlywheelCommand() {
    return this.startEnd(() -> setShooterOn(), () -> setShooterOff()).withName("Spinning Up Flywheel");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command runFeederCommand() {
    return this.startEnd(() -> setShooterOn(), () -> setShooterOff()).withName("Feeding");
  }

  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches
   * the desired speed it starts the Feeder.
   */
  public Command runShooterCommand() {
    return this.startEnd(() -> setShooterOn(), () -> setShooterOff())
        .until(isFlywheelSpinning)
        .andThen(this.startEnd(() -> setShooterOn(), () -> setShooterOff()))
        .withName("Shooting");
  }

  public void setShooter(boolean enabled) {
    if (enabled) {
      setShooterOn();
    } else {
      setShooterOff();
    }
  }

  @Override
  public void periodic() {
    // Display subsystem values
    SmartDashboard.putNumber("Shooter | Flywheel | Commanded Output", flywheelTargetOutput);
    SmartDashboard.putNumber("Shooter | Flywheel | Actual Output", flywheelMotor.get());
    // Supply current (may not exist in older Phoenix versions)
    try {
      // Removed: supply current display (getSupplyCurrent not available)
    } catch (Throwable ignored) {
    }

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
  }
}