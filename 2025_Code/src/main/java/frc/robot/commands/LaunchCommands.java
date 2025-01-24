// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public final class LaunchCommands {
  private LaunchCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command launchNoteAuto(LauncherSubsystem _Launcher, IntakeSubsystem _Intake, TurretSubsystem _Turret) {
    Command launching =
        new Command() {
          
          private Timer m_timer;
          private int CurrentTurretPosition;
          //private double saveRightCmdRPM;
          //private double saveLeftCmdRPM;

          @Override
          public void initialize() {
            /* Start the Launcher Wheels and the Launch timer. */
            CurrentTurretPosition = _Turret.getSelPosition();
            SmartDashboard.putNumber("launcherAuto curr Turret Pos", CurrentTurretPosition);

            if (CurrentTurretPosition == Constants.Turret.kAmpID) {
              _Launcher.SetLCWR(Constants.Launcher.kAmpL);
              _Launcher.SetRCWR(Constants.Launcher.kAmpR);
            }
            else if (CurrentTurretPosition == Constants.Turret.kHighShotID) {
              _Launcher.SetLCWR(Constants.Launcher.kHighShotL);
              _Launcher.SetRCWR(Constants.Launcher.kHighShotR);
            }
            else if (CurrentTurretPosition == Constants.Turret.kStartID) {
              _Launcher.SetLCWR(Constants.Launcher.kStartL);
              _Launcher.SetRCWR(Constants.Launcher.kStartR);
            }

            _Launcher.startFlyWheels();
            m_timer = new Timer();
            m_timer.start();
            _Launcher.periodic();
            _Intake.periodic();
          }

          @Override
          public void execute() {
            /* Wait until the Launcher Wheels get up to speed,
             * then start the Intake Feeder to push the Note up
             * into the Launcher Wheels.
             */
            _Launcher.startFlyWheels();

             if(m_timer.get() > Constants.Launcher.kTimeToLaunch){
              _Intake.moveNote(Constants.Launcher.kFeederSpeed);
              _Intake.periodic();
            }

            _Launcher.periodic();
          }

          @Override
          public boolean isFinished() {
            /* The Launcher and Intake feeder will stop after an
             * appropriate delay.
             */
            return m_timer.get() > Constants.Launcher.kTimeToStop;
          }

          @Override
          public void end(boolean interrupted) {
            /* Stop both the Launcher and the Intake feeder */
            _Launcher.stopFlyWheels();
            _Intake.stopFeeder();
            _Launcher.periodic();
            _Intake.periodic();
          }
        };

        return launching;
  }
}
