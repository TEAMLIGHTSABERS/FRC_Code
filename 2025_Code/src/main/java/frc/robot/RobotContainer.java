// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath; //Only needed for on-the-fly pathplanning

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();

  // A chooser for autonomous commands
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController).getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController).getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController).getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0, 0.0), m_intake));

    // configure the launcher to stop when no other command is running
    m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

    m_turret.setDefaultCommand(new RunCommand(() -> m_turret.driveWench(
        (m_driverController).getRightBumperPressed(),
        ((m_driverController).getRightTriggerAxis() > Constants.OIConstants.kTriggerButtonThreshold)),
        m_turret));
/*
    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Straight Auto", m_StraightAuto);
    m_chooser.addOption("Red Auto", m_redLeftAuto);
    m_chooser.addOption("Blue Auto", m_blueRightAuto);
    m_chooser.addOption("Center Auto", m_centerAuto);
    m_chooser.addOption("Test Auto", m_testAuto);
 
    SmartDashboard.putData("Auto Selection", m_chooser);
    SmartDashboard.putData("Launcher Commands", m_launcher);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    // Put subsystems to dashboard.
    Shuffleboard.getTab("Drivetrain").add("Commands", m_robotDrive);
    Shuffleboard.getTab("Intake Subsystem").add("Commands", m_intake);
    Shuffleboard.getTab("Launcher Subsystem").add("Commands", m_launcher);
    Shuffleboard.getTab("Turret Subsystem").add("Commands", m_turret);
    */
/*
    autoChooser.addOption("Test Auto", m_testAuto);
    SmartDashboard.putData("Auto Selection", autoChooser);
    SmartDashboard.putData("Launcher Commands", m_launcher);
    */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    // button to put swerve modules in an "x" configuration to hold position
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Turret Controls ----------------------------------------------------------
    // Right Trigger will Raise the Launcher elevation.    
//    new Trigger(
//            () -> ((XboxController) m_driverController).getRightTriggerAxis()
//            > Constants.OIConstants.kTriggerButtonThreshold)
///        .onTrue(new RunCommand(() -> m_turret.reducePOS()));

    // Right Bumper will Lower the Launcher elevation
//    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
//        .onTrue(new RunCommand(() -> m_turret.advancePOS()));

    // Intake Controls ----------------------------------------------------------
    // Left Trigger will start the Intake and pickup a Note
    new Trigger(
            () ->
                ((XboxController) m_driverController).getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kTopPower, Constants.Intake.kFeedPower), m_intake))
        .onFalse(new RunCommand(() -> m_intake.setPower(0.0, 0.0), m_intake));

    // "B" Button will slowly "Backup" a Note in the Intake
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(0.0, -0.2), m_intake))
        .onFalse(new RunCommand(() -> m_intake.setPower(0.0, 0.0), m_intake));

    // Launcher Controls -------------------------------------------------------
    // "A" Button will launch a Note toward the Target
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(m_launcher.launchNote(m_intake, m_turret));
    
    // "X" Button will run the Launcher Flywheels for 5 seconds
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(m_launcher.testFlyWheels());

    SmartDashboard.putData("New Auto", new PathPlannerAuto("New Auto"));
  }

 /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return m_chooser.getSelected();
/*    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    Commented out because it is part of 2025 year update. (Updated library not released at time of editing)
*/

    return autoChooser.getSelected();
  }
}
