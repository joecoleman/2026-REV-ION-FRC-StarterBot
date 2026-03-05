// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class Autos {

  public static Command exampleAuto(DriveSubsystem drive) {
    // Create config for trajectory
        // This line was corrected from 'ublic' to 'public'
        TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // Additional context for the method
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            drive::setModuleStates,
            drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0, false));
  }

  public static Command shootAuto(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      FeederSubsystem feeder
  ) {
    // Trajectory: drive ahead 1 foot (0.3048 meters)
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(0.3048, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand driveForwardCommand = new SwerveControllerCommand(
        trajectory,
        drive::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(trajectory.getInitialPose());

    return new SequentialCommandGroup(
        driveForwardCommand.andThen(() -> drive.drive(0, 0, 0, false)),
        // Shooter on for 2 seconds
        new WaitCommand(0.1),
        new edu.wpi.first.wpilibj2.command.InstantCommand(() -> shooter.setShooter(true), shooter),
        new WaitCommand(2.0),
        // Feeder on after shooter runs for 2 seconds
        new edu.wpi.first.wpilibj2.command.InstantCommand(() -> feeder.setFeeder(true), feeder),
        // Wait until 10 seconds total, then stop both
        new WaitCommand(8.0),
        new edu.wpi.first.wpilibj2.command.InstantCommand(() -> shooter.setShooter(false), shooter),
        new edu.wpi.first.wpilibj2.command.InstantCommand(() -> feeder.setFeeder(false), feeder)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
