// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public final class IntakeCommands {
  private IntakeCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }


  /**
* Constructs a command that starts the Intake.
*
* @return The PickUp command
*/

public static Command pickupNoteAuto(IntakeSubsystem _Intake) {
    Command pickingUp =
        new Command() {
          
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
            _Intake.start();
            _Intake.periodic();
          }

          @Override
          public void execute() {
            _Intake.periodic();
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Launcher.kTimeToStop;
          }

          @Override
          public void end(boolean interrupted) {
            _Intake.stop();
            _Intake.periodic();
          }
        };

        return pickingUp;
  }

}
