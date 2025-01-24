// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public final class TurretCommands {
  private TurretCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command rotateBodyAzCommand(DriveSubsystem drivesys, double cmdedFieldAz) {
    Command rotateTo =
        new Command() {
          
          @Override
          public void initialize() {
          }

          @Override
          public void execute() {
            double currAz = drivesys.getHeading();
            
            if (currAz < cmdedFieldAz){
              drivesys.drive(0.0, 0.0, 0.1, true, false);
            } else {
              drivesys.drive(0.0, 0.0, -0.1, true, false);
            }
          }

          @Override
          public boolean isFinished() {
            double currAz = drivesys.getHeading();
            /* The Launcher and Intake feeder will stop after an
             * appropriate delay.
             */
            Boolean reachedDeadband = Math.abs
              (
                MathUtil.applyDeadband
                (
                  currAz - cmdedFieldAz,
                  Constants.OIConstants.kDriveAngleDeadband
                )
              ) < Constants.OIConstants.kDriveAngleDeadband;

            return (reachedDeadband);
          }

          @Override
          public void end(boolean interrupted) {
            /* Stop both the Launcher and the Intake feeder */
            drivesys.drive(0.0, 0.0, 0.0, true, false);
          }
        };

    return rotateTo;
  }

  public static Command raiseTurretCommand(TurretSubsystem turretSys, int cmdedTurretPos) {
    Command raiseTo =
        new Command() {
          
          @Override
          public void initialize() {

            int currSelPos = turretSys.getSelPosition();
            while(currSelPos != cmdedTurretPos)
            {
              if (currSelPos < cmdedTurretPos) {
                turretSys.advancePOS();
              } else if (currSelPos > cmdedTurretPos) {
                turretSys.reducePOS();
              }

              currSelPos = turretSys.getSelPosition();
            };
          }

          @Override
          public boolean isFinished() {
            return (true);
          }
        };

    return raiseTo;
  }

  public static Command timeDelay(double delayTime) {
    Command Delay =
        new Command() {
          private Timer m_timer;
        
          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Launcher.kTimeToStop;
          }
        };

    return Delay;
  }


}
