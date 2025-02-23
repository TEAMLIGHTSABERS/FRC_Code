// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
 
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevatorSubSystem = new ElevatorSubsystem();
  
  // A chooser for autonomous commands
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  private static SendableChooser<Command> autoChooser;
  
  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("Coral Station", m_elevatorSubSystem.setSetpointCommand(Setpoint.kFeederStation));
    NamedCommands.registerCommand("Level 2", m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel2));
    NamedCommands.registerCommand("Level 3", m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel3));
    NamedCommands.registerCommand("Level 4", m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel4));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoTab.add(autoChooser).withPosition(1,1);
    
    //SmartDashboard.putData("Auto Chooser", autoChooser);
    //SmartDashboard.putData("New Auto", new PathPlannerAuto("New Auto"));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
                    m_robotDrive));  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Right Stick Button -> drive to April tag
    /*if(m_driverController.rightStick()){

    }*/
    
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    // A Button -> Elevator/Wrist to level 2 position
    m_driverController.a().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // B Button -> Elevator/Wrist to level 3 position
    m_driverController.b().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Wrist to level 4 position
    m_driverController.y().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kLevel4));

    // Right Bumper -> Run tube intake
    m_driverController.rightBumper().whileTrue(m_elevatorSubSystem.runIntakeCommand());

    // Right Trigger  -> Elevator/Wrist to human player position
    m_driverController.rightTrigger(OIConstants.kTriggerButtonThreshold)
      .onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kFeederStation));

    // Left Bumper -> Wrist to Driver Input
    m_driverController.leftBumper().whileTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kWDriverInput));

    /**Extra Button Commands
      // Right Bumper -> Elevator to Driver Input
    //m_driverController.rightBumper().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kDriverInput));

    // Right Trigger  -> Wrist to Driver Input
    //m_driverController.rightTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(m_elevatorSubSystem.setSetpointCommand((Setpoint.kDriverInput)));

    // Left Bumper -> Run tube intake
    //m_driverController.leftBumper().whileTrue(m_elevatorSubSystem.forwardIntakeCommand());

    // Left Trigger -> Run ball intake in reverse, set to stow when idle
    //m_driverController.leftTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(m_elevatorSubSystem.reverseIntakeCommand());

    // B Button -> Elevator/Wrist to human player position, set ball intake to stow
    // when idle    
    //m_driverController
    //    .b().onTrue(m_elevatorSubSystem.setSetpointCommand(Setpoint.kFeederStation));*/
  }
     

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {  
    return autoChooser.getSelected();
  }
}