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

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
