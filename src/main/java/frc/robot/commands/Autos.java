// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
 
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class Autos {

  

    

    
   
  

  public static Command shootAuto(
    DriveSubsystem drive,
    ShooterSubsystem shooter,
    FeederSubsystem feeder,
    IntakeSubsystem intake
  ) {
  // No initial trajectory: start directly with the timed backward drive

  return new SequentialCommandGroup(
    // Drive backwards for 1 second (instead of waiting) before starting the shooter
    new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(-0.15, 0, 0, false), drive)
        .withTimeout(1.0)
        .andThen(() -> drive.drive(0, 0, 0, false)),
    // Shooter on for 2 seconds
    new edu.wpi.first.wpilibj2.command.InstantCommand(() -> shooter.setShooter(true), shooter),
    new WaitCommand(2.0),
    // Run feeder and intake together for 8 seconds
    new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {}, feeder, intake),
    new edu.wpi.first.wpilibj2.command.ParallelCommandGroup(
      feeder.runFeederCommand(),
      intake.runIntakeCommand()
    ).withTimeout(8.0),
    // Stop shooter
    new edu.wpi.first.wpilibj2.command.InstantCommand(() -> shooter.setShooter(false), shooter)
  );
  }

  @SuppressWarnings("unused")
  public static Command complexAuto(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      IntakeSubsystem intake
  ) {
    // Conversion constants
    final double INCHES_TO_METERS = 0.0254;
    // 150 inches forward
    double forward1 = 150 * INCHES_TO_METERS;
    // 50 inches backward
    double backward = 50 * INCHES_TO_METERS;
    // 1 meter forward
    double forward2 = 1.0;

    // Sequence
    return new SequentialCommandGroup(
      // Drive forward 150 inches
      new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(3.0),
      new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
      // Wait 3 seconds
      new WaitCommand(3.0),
      // Back up 50 inches
      new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(-0.5, 0, 0, false), drive).withTimeout(1.0),
      new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
      // Turn 90 degrees right
      new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(0, 0, -0.5, false), drive).withTimeout(1.0),
      new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
      // Drive forward 1 meter
      new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(0.5, 0, 0, false), drive).withTimeout(1.0),
      new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
      // Turn 90 degrees right again
      new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(0, 0, -0.5, false), drive).withTimeout(1.0),
      new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive),
      // Activate shooter
      new edu.wpi.first.wpilibj2.command.InstantCommand(() -> shooter.setShooter(true), shooter),
      // Wait 3 seconds
      new WaitCommand(3.0),
      // Run feeder and intake for 10 seconds
      new edu.wpi.first.wpilibj2.command.ParallelCommandGroup(
        feeder.runFeederCommand(),
        intake.runIntakeCommand()
      ).withTimeout(10.0),
      // Stop shooter
      new edu.wpi.first.wpilibj2.command.InstantCommand(() -> shooter.setShooter(false), shooter)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
