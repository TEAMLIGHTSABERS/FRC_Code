// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
/*

  // Autonoumous Command for driving from the center of the Speaker.
  public static Command centerAuto
  (
    DriveSubsystem driveSys,
    LauncherSubsystem launchSys,
    IntakeSubsystem intakeSys,
    TurretSubsystem turretSys
  )
  {
    return Commands.sequence(
      // This command assumes that the robot is aligned to 0 heading alon+x axis pointing towards the opposing
      // Alliance station and the Robot is rotated to 180 degrees before the team leaves the field.
      // Starting position (0, 0, 180).
      TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kHighShotID),   // Raise Turret to Speaker Shot Position
      TurretCommands.timeDelay(3),                                        // Time delay prevents Turret & LaunchNote interference 
      LaunchCommands.launchNoteAuto(launchSys, intakeSys, turretSys),               // Shoot Note into Speaker
      TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kStartID),      // Raise Turret to Speaker Shot Position
      DriveCommands.straightAutoCommand11(driveSys),                                // move out of Zone to position (1.0, 0) m.                
    
      // More Advanced CenterAuto pick up a second Note and shoots it.
      IntakeCommands.pickupNoteAuto(intakeSys),                                     // Pickup the Note
      DriveCommands.straightAutoCommand12(driveSys),                                // move back to speaker (0, 0 180) m.                
      TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kHighShotID),   // Raise Turret to Speaker Shot Position
      TurretCommands.timeDelay(3),                                        // Time delay prevents Turret & LaunchNote interference 
      LaunchCommands.launchNoteAuto(launchSys, intakeSys, turretSys),               // Shoot Note into Speaker
      TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kStartID),      // Raise Turret to Speaker Shot Position
      DriveCommands.straightAutoCommand11(driveSys));                               // move out of Zone to position (1.0, 0) m.                
  }

  // Autonoumous Command for driving from the left side of the Speaker on the Red Alliance.
  public static Command redLeftAuto
  (
    DriveSubsystem driveSys,
    LauncherSubsystem launchSys,
    IntakeSubsystem intakeSys,
    TurretSubsystem turretSys
  )
  {
    return Commands.sequence(
      // Assumes that you align 0 heading along +x towards the opposing Alliance
      // and Robot get rotated to -135 degrees before the team leaves the field.
      // Starting position (0, 0, -135).
      TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kHighShotID),  // Raise Turret to Speaker Shot Position
      TurretCommands.timeDelay(3),                                       // Time delay prevents Turret & LaunchNote interference
      LaunchCommands.launchNoteAuto(launchSys, intakeSys, turretSys),              // Shoot Note into Speaker
      TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kStartID),     // Drop the Turret to the Start Position
      DriveCommands.straightAutoCommand21(driveSys));                              // move out of Zone to position (3, 3,  -135).                

      // More Advanced redLeftAuto picks up a second Note and shoots it.
      //DriveCommands.straightAutoCommand21Alt(driveSys));                            // moves to Field Note at (2, 1, -180).                
      //IntakeCommands.pickupNoteAuto(),                                              // Pickup the Note
      //DriveCommands.straightAutoCommand13(driveSys),                                // move back to speaker (0, 0, -135).                
      //TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kHighShotID),   // Raise Turret to Speaker Shot Position
      //LaunchCommands.launchNoteAuto(launchSys, intakeSys, turretSys));              // Shoot Note into Speaker
      //TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kStartID),      // Drop the Turret to the Start Position
  }

  // Autonoumous Command for driving from the right side of the Speaker on the Blue Alliance.
  public static Command blueRightAuto
  (
    DriveSubsystem driveSys,
    LauncherSubsystem launchSys,
    IntakeSubsystem intakeSys,
    TurretSubsystem turretSys
  )
  {
    return Commands.sequence(
      // This command assumes that the robot is aligned to 0 heading along +x axis pointing towards the Red
      // Alliance station.  The Robot is rotated to 135 degrees and placed against the right side of the 
      // Speaker before the team leaves the field.
      // Starting position (0, 0, 135).

      TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kHighShotID),   // Raise Turret to Speaker Shot Position
      TurretCommands.timeDelay(3),                                        // Time delay prevents Turret & LaunchNote interference
      LaunchCommands.launchNoteAuto(launchSys, intakeSys, turretSys),               // Shoot Note into Speaker
      TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kStartID),      // Drop the Turret to the Start Position
      DriveCommands.straightAutoCommand31(driveSys));                               // move out of Zone to position (3, -3,  135).                

      // More Advanced BlueRightAuto picks up a second Note and shoots it.
      //DriveCommands.straightAutoCommand31Alt(driveSys));                            // moves to Field Note at (2, -1, 180).                
      //IntakeCommands.pickupNoteAuto(),                                              // Pickup the Note
      //DriveCommands.straightAutoCommand13(driveSys),                                // move back to speaker (0, 0, 135).                
      //TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kHighShotID),   // Raise Turret to Speaker Shot Position
      //TurretCommands.launchNoteAuto(launchSys, intakeSys, turretSys));              // Shoot Note into Speaker
      //TurretCommands.raiseTurretCommand(turretSys, Constants.Turret.kStartID),      // Drop the Turret to the Start Position
  }

  public static Command testAuto
  (
    DriveSubsystem driveSys
  )
  {
    return Commands.sequence(
      // Alliance station.  The Robot is rotated to -90 degrees.
      // Starting position (0, 0, -90).
      DriveCommands.autoTest1(driveSys, 4, 4));                               // move out of Zone to position (4, 4, -90).                

  }

  public static Command straightAutoTest
  (
    DriveSubsystem driveSys
  )
  {
    return Commands.sequence(
      // Alliance station.  The Robot is rotated to 0 degrees.
      // Starting position (0, 0, 0).
      DriveCommands.straightAutoCommand1(driveSys, 2.5, 0));                               // move out of Zone to position (2.5, 0, 0).                

  }
*/
}
