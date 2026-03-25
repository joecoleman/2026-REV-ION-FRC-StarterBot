// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeLiftSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final IntakeLiftSubsystem m_intakeLift = new IntakeLiftSubsystem();

  // The driver's controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  // Operator's controller (second controller)
  private final CommandXboxController m_operatorController =
    new CommandXboxController(OIConstants.kOperatorControllerPort);

  // Autonomous chooser
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
  m_robotDrive.setDefaultCommand(
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    new RunCommand(
      () -> {
        // Default to 1/3 speed, left bumper = 3/4 speed
        double speedScale = m_driverController.getHID().getLeftBumper() ? 0.75 : 0.33;
        m_robotDrive.drive(
          -MathUtil.applyDeadband(
            m_driverController.getLeftY(), OIConstants.kDriveDeadband) * speedScale,
          -MathUtil.applyDeadband(
            m_driverController.getLeftX(), OIConstants.kDriveDeadband) * speedScale,
          -MathUtil.applyDeadband(
            m_driverController.getRightX(), OIConstants.kDriveDeadband) * speedScale,
          true);
      },
      m_robotDrive).withName("Robot Drive Default"));

    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_feeder);

    SmartDashboard.putNumber("Bat Voltage", RobotController.getBatteryVoltage());

    SmartDashboard.putData("Intake", m_intake.runIntakeCommand().withName("Intake - Intaking"));
    SmartDashboard.putData("Extake", m_intake.runExtakeCommand().withName("Intake - Extaking"));

    SmartDashboard.putData("Feeder", m_feeder.runFeederCommand().withName("Feeder - Feeding"));
    SmartDashboard.putData("Flywheel", m_shooter.runFlywheelCommand().withName("Shooter - Spinning up Flywheel"));

    // Autonomous chooser setup
  m_autoChooser.setDefaultOption("Taco Tuesday", Autos.shootAuto(m_robotDrive, m_shooter, m_feeder, m_intake));
  m_autoChooser.addOption("Chips And Salsa", Autos.simpleAuto(m_robotDrive, m_shooter, m_feeder, m_intake));
  m_autoChooser.addOption("The Whole Enchilada Auto", Autos.driveThenShootAuto(m_robotDrive, m_shooter, m_feeder, m_intake));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Left Stick Button -> Set swerve to zero heading
    m_driverController.leftStick().whileTrue(m_robotDrive.zeroHeadingCommand());

    // toggle intake on/off with A button
    m_driverController.a().toggleOnTrue(m_intake.runIntakeCommand());
    // A button -> Run flywheel while held
    m_operatorController
      .a()
      .whileTrue(m_shooter.runFlywheelCommand());
    // Right Trigger -> Run fuel intake in reverse
    m_driverController
      .rightTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(m_intake.runIntakeCommand());

    // Left Trigger -> Run fuel intake in reverse
    m_driverController
      .leftTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(m_intake.runExtakeCommand());

    // B button -> Raise intake lift while held
    m_driverController.b().whileTrue(new RunCommand(() -> m_intakeLift.setLiftPower(0.5), m_intakeLift));
    // A button -> Lower intake lift while held
    m_driverController.a().whileTrue(new RunCommand(() -> m_intakeLift.setLiftPower(-0.5), m_intakeLift));
    // Stop lift when button released is handled by command ending

    // Operator controller bindings
    // Y -> Toggle shooter on/off (operator)
    m_operatorController.leftBumper().toggleOnTrue(m_shooter.runShooterCommand());
    // X -> Run feeder while held (operator)
    m_operatorController.rightBumper().whileTrue(m_feeder.runFeederCommand().withName("Feeder - Operator"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
