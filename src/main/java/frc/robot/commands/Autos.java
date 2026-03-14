// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.concurrent.atomic.AtomicReference;
import edu.wpi.first.math.geometry.Pose2d;
 
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
  // No initial trajectory: start by driving backwards 1 meter (odometry-based)
  double driveFraction = 0.15; // fraction of max linear speed for the backwards motion
  AtomicReference<Pose2d> startPose = new AtomicReference<>();

  Command backOneMeter = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> startPose.set(drive.getPose()))
      .andThen(new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(-driveFraction, 0, 0, false), drive)
          .until(() -> drive.getPose().getTranslation().getDistance(startPose.get().getTranslation()) >= 1.0))
      .andThen(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive));

  return new SequentialCommandGroup(
    // Drive backwards 1 meter before starting the shooter
    backOneMeter,
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



  public static Command simpleAuto(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      IntakeSubsystem intake
  ) {
  // Use odometry-based 'run until' commands so the robot stops when the pose/heading target is reached.
  double driveFraction = 0.10; // fraction of max linear speed to use for translation
  double rotFraction = 0.10; // fraction of max angular speed to use for rotation

  double forwardMeters = 2.60;
  double backwardMeters = 1.0;
  double turnDegrees = 105.0; // right turn

  // Holders for start pose/heading
  AtomicReference<Pose2d> startPose = new AtomicReference<>();
  AtomicReference<Double> startHeading = new AtomicReference<>();

  // Drive forward until distance reached
  Command forwardCmd = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> startPose.set(drive.getPose()))
    .andThen(new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(driveFraction, 0, 0, false), drive)
      .until(() -> drive.getPose().getTranslation().getDistance(startPose.get().getTranslation()) >= forwardMeters))
    .andThen(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive));

  // Back up until distance reached
  Command backCmd = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> startPose.set(drive.getPose()))
    .andThen(new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(-driveFraction, 0, 0, false), drive)
      .until(() -> drive.getPose().getTranslation().getDistance(startPose.get().getTranslation()) >= backwardMeters))
    .andThen(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive));

  // Turn right until heading changed by ~90 degrees
  Command turnCmd = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> startHeading.set(drive.getHeading()))
    .andThen(new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(0, 0, -rotFraction, false), drive)
      .until(() -> {
        double delta = Math.IEEEremainder(drive.getHeading() - startHeading.get(), 360.0);
        return Math.abs(delta) >= Math.abs(turnDegrees);
      }))
    .andThen(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive));

  return new SequentialCommandGroup(
    forwardCmd,
    new WaitCommand(3.0),
    backCmd,
    turnCmd);
  }

  /**
   * Drive sequence like simpleAuto, then shoot: after the turn activate the shooter for 3s,
   * then run feeder+intake for 8s.
   */
  public static Command driveThenShootAuto(
    DriveSubsystem drive,
    ShooterSubsystem shooter,
    FeederSubsystem feeder,
    IntakeSubsystem intake) {

  double driveFraction = 0.10; // fraction of max linear speed to use for translation
  double rotFraction = 0.10; // fraction of max angular speed to use for rotation

  double forwardMeters = 2.60;
  double backwardMeters = 1.0;
  double turnDegrees = 105.0; // right turn (match simpleAuto)

  AtomicReference<Pose2d> startPose = new AtomicReference<>();
  AtomicReference<Double> startHeading = new AtomicReference<>();

  Command forwardCmd = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> startPose.set(drive.getPose()))
    .andThen(new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(driveFraction, 0, 0, false), drive)
      .until(() -> drive.getPose().getTranslation().getDistance(startPose.get().getTranslation()) >= forwardMeters))
    .andThen(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive));

  Command backCmd = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> startPose.set(drive.getPose()))
    .andThen(new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(-driveFraction, 0, 0, false), drive)
      .until(() -> drive.getPose().getTranslation().getDistance(startPose.get().getTranslation()) >= backwardMeters))
    .andThen(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive));

  Command turnCmd = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> startHeading.set(drive.getHeading()))
    .andThen(new edu.wpi.first.wpilibj2.command.RunCommand(() -> drive.drive(0, 0, -rotFraction, false), drive)
      .until(() -> {
        double delta = Math.IEEEremainder(drive.getHeading() - startHeading.get(), 360.0);
        return Math.abs(delta) >= Math.abs(turnDegrees);
      }))
    .andThen(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> drive.drive(0, 0, 0, false), drive));

  // Shooter sequence: after the turn activate shooter for 3s, then run feeder+intake for 8s
  Command shooterSequence = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> shooter.setShooter(true), shooter)
    .andThen(new WaitCommand(3.0))
    .andThen(new edu.wpi.first.wpilibj2.command.ParallelCommandGroup(
      feeder.runFeederCommand(),
      intake.runIntakeCommand()
    ).withTimeout(8.0))
    .andThen(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> shooter.setShooter(false), shooter));

  return new SequentialCommandGroup(
    forwardCmd,
    new WaitCommand(3.0),
    backCmd,
    turnCmd,
    shooterSequence);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
