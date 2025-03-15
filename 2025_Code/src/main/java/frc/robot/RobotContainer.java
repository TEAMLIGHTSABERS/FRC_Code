// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorSubsystemConstants.IntakeSetpoints;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Translation2d;
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
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  
  // A chooser for autonomous commands
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  private static SendableChooser<Command> autoChooser;

  private Translation2d RightOffset = new Translation2d(AprilTagConstants.kXATright, AprilTagConstants.kYAT);
  private Translation2d LeftOffset = new Translation2d(AprilTagConstants.kXATleft, AprilTagConstants.kYAT);
  
  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_robotDrive.setupPathPlannerRobot();

    NamedCommands.registerCommand("Coral Station", m_elevatorSubsystem.setSetpointCommand(Setpoint.kFeederStation));
    NamedCommands.registerCommand("Level 2", m_elevatorSubsystem.setSetpointCommand(Setpoint.kLevel2));
    NamedCommands.registerCommand("Level 3", m_elevatorSubsystem.setSetpointCommand(Setpoint.kLevel3));
    NamedCommands.registerCommand("Level 4", m_elevatorSubsystem.setSetpointCommand(Setpoint.kLevel4));
    
    NamedCommands.registerCommand("Coral In", m_elevatorSubsystem.autoIntakeCommand(IntakeSetpoints.kReverse, 2.0));
    NamedCommands.registerCommand("Coral Out", m_elevatorSubsystem.autoIntakeCommand(IntakeSetpoints.kForward, 2.0));
        
    /*autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoTab.add(autoChooser).withPosition(1,1);*/
    
    //SmartDashboard.putData("Auto Chooser", autoChooser);
    //SmartDashboard.putData("New Auto", new PathPlannerAuto("New Auto"));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    /*m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
                    m_robotDrive));*/
                    
    // Configure default commands
      m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () ->
              m_robotDrive.drive(
                  -m_robotDrive.applyDeadbandAndCurve(m_driverController.getLeftY(), OIConstants.kDriveDeadband, OIConstants.kDriveExponent),
                  -m_robotDrive.applyDeadbandAndCurve(m_driverController.getLeftX(), OIConstants.kDriveDeadband, OIConstants.kDriveExponent),
                  -m_robotDrive.applyDeadbandAndCurve(m_driverController.getRightX(), OIConstants.kDriveDeadband, OIConstants.kDriveExponent),
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
    //m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    // A Button -> Elevator/Wrist to level 2 position
    m_driverController.a().onTrue(m_elevatorSubsystem.setSetpointCommand(Setpoint.kLevel2));

    // B Button -> Elevator/Wrist to level 3 position
    m_driverController.b().onTrue(m_elevatorSubsystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Wrist to level 4 position
    m_driverController.y().onTrue(m_elevatorSubsystem.setSetpointCommand(Setpoint.kLevel4));


    // Right Bumper -> Run coral intake when you press the button and when you let go it puts the elevator down and the wrist up
    m_driverController.rightBumper().whileTrue(m_elevatorSubsystem.runIntakeCommand()).onFalse(m_elevatorSubsystem.setSetpointCommand(Setpoint.kFeederStation));

    // Right Trigger  -> Elevator/Wrist to human player position
    m_driverController.rightTrigger(OIConstants.kTriggerButtonThreshold)
      .onTrue(m_elevatorSubsystem.setSetpointCommand(Setpoint.kFeederStation));

    // X Button -> Driver Input
    m_driverController.x().onTrue(m_elevatorSubsystem.setSetpointCommand(Setpoint.kDriverInput));

    // Left Trigger -> Climb Down
    m_driverController.leftTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(m_climbSubsystem.climbDownCommand());

    // Left Bumper -> Climb Up
    m_driverController.leftBumper().whileTrue(m_climbSubsystem.climbUpCommand());

    // Right Stick Button -> Move to right reef relative to the April Tag
    m_driverController.rightStick().onTrue(
      new RunCommand (() -> 
      {if (!m_VisionSubsystem.isAtTarget(RightOffset)) {
        m_VisionSubsystem.moveToAprilTag(RightOffset);
      } else {
        m_VisionSubsystem.stop();
        }
      })
    );

    // Left Stick Button -> Move to left reef relative to the April Tag
    m_driverController.leftStick().onTrue(
      new RunCommand (() -> 
      {if (!m_VisionSubsystem.isAtTarget(LeftOffset)) {
        m_VisionSubsystem.moveToAprilTag(LeftOffset);
      } else {
        m_VisionSubsystem.stop();
        }
      })
    );

    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoTab.add(autoChooser).withPosition(1,1);
    
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