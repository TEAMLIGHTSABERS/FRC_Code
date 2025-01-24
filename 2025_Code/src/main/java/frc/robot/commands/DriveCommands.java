// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class DriveCommands {
  private DriveCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  //This is the basic command used to move the aligned robot along a stright line any direction having a +X.
  public static Command straightAutoCommand1(DriveSubsystem drivesys, double xPos, double yPos) {
    // This command assumes that the robot is aligned to 0 heading along +x axis pointing towards the opposing
    // Alliance station and the Robot is not moved from its aligned position.
    // Starting position (0, 0, 0).

    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPos/3, yPos/3), new Translation2d(xPos*2/3, yPos*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xPos, yPos, new Rotation2d(Math.toRadians(0))), config
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    straightTrajectory,
    drivesys::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    drivesys::setModuleStates,
    drivesys);

    // Reset odometry to the starting pose of the trajectory.
    drivesys.resetOdometry(straightTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivesys.drive(0, 0, 0, false, false));
  }

  // This is the first movement command for the CenterAuto operation sequence.
  // Version 11 moves out of the zone by 2 m while faceing 180 degrees.
  public static Command straightAutoCommand11(DriveSubsystem drivesys) {
    // This command assumes that the robot is aligned to 0 heading along +x axis pointing towards the opposing
    // Alliance station and the Robot is rotated to 180 degrees before the team leaves the field.
    // Starting position (0, 0, 180).

    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double xPos = 1.5;
    double yPos = 0.0; 

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPos/3, yPos/3), new Translation2d(xPos*2/3, yPos*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xPos, yPos, new Rotation2d(Math.toRadians(180))), config
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    straightTrajectory,
    drivesys::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    drivesys::setModuleStates,
    drivesys);

    // Reset odometry to the starting pose of the trajectory.
    drivesys.resetOdometry(straightTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivesys.drive(0, 0, 0, false, false));
  }

  // This is the second movement command for the CenterAuto operation sequence.
  // Version 12 moves back to the Speaker while faceing 180 degrees.
  public static Command straightAutoCommand12(DriveSubsystem drivesys) {
    // This command assumes that the robot is located at (2, 0, 180) 

    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double startXPos = 1.0;
    double startYPos = 0.0;
    double finishXPos = 0.0;
    double finishYPos = 0.0;
    double deltaXPos = finishXPos-startXPos;
    double deltaYPos = finishYPos-startYPos;
    double xPosP1 = startXPos + deltaXPos/3;
    double xPosP2 = startXPos + deltaXPos*2/3;
    double yPosP1 = startYPos + deltaYPos/3;
    double yPosP2 = startYPos + deltaYPos*2/3;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(startXPos, startYPos, new Rotation2d(Math.toRadians(180))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPosP1, yPosP1),
                new Translation2d(xPosP2, yPosP2)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(finishXPos, finishYPos, new Rotation2d(Math.toRadians(180))), config
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    straightTrajectory,
    drivesys::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    drivesys::setModuleStates,
    drivesys);

    // Reset odometry to the starting pose of the trajectory.
    drivesys.resetOdometry(straightTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivesys.drive(0, 0, 0, false, false));
  }

  // This is the first movement command for the redLeftAuto operation sequence.
  // Version 21 moves out of the zone along the -45 degree vector by moving 3m.
  // The robot orientation remains at -135 degrees for whole the movement.
  public static Command straightAutoCommand21(DriveSubsystem drivesys) {
    // This command assumes that the robot is located at (0, 0, -135) 

    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double xPos = 3.0;
    double yPos = 3.0; 

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(-135))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPos/3, yPos/3), new Translation2d(xPos*2/3, yPos*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xPos, yPos, new Rotation2d(Math.toRadians(-135))), config
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    straightTrajectory,
    drivesys::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    drivesys::setModuleStates,
    drivesys);

    // Reset odometry to the starting pose of the trajectory.
    drivesys.resetOdometry(straightTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivesys.drive(0, 0, 0, false, false));
  }

  // This is the first movement command for the redLeftAuto operation sequence.
  // Version 31 moves out of the zone along the +45 degree vector by moving 3m.
  // The robot orientation remains at +135 degrees for whole the movement.
  public static Command straightAutoCommand31(DriveSubsystem drivesys) {
    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double xPos = 3.0;
    double yPos = -3.0; 

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(135))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPos/3, yPos/3), new Translation2d(xPos*2/3, yPos*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xPos, yPos, new Rotation2d(Math.toRadians(135))), config
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    straightTrajectory,
    drivesys::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    drivesys::setModuleStates,
    drivesys);

    // Reset odometry to the starting pose of the trajectory.
    drivesys.resetOdometry(straightTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivesys.drive(0, 0, 0, false, false));
  }

  //This is the basic command used to move the aligned robot along a stright line any direction having a +X.
  public static Command autoTest1(DriveSubsystem drivesys, double xPos, double yPos) {
    // This command assumes that the robot is aligned to 0 heading along +x axis pointing towards the opposing
    // Alliance station and the Robot is not moved from its aligned position.
    // Starting position (0, 0, -90).

    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPos/4, yPos/2), new Translation2d(xPos*3/4, yPos/2)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xPos, yPos, new Rotation2d(Math.toRadians(-90))), config
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    straightTrajectory,
    drivesys::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    drivesys::setModuleStates,
    drivesys);

    // Reset odometry to the starting pose of the trajectory.
    drivesys.resetOdometry(straightTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivesys.drive(0, 0, -90, false, false));
  }

}
