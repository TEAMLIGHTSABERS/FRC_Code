package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.ClimbSubsystemConstants.ClimbSetpoints;



public class ClimbSubsystem extends SubsystemBase{


    /*Intake Motor Initialization */
    private SparkMax climbMotor =
        new SparkMax(ClimbSubsystemConstants.kClimbMotorCanId, MotorType.kBrushless);

    public ClimbSubsystem() {

        climbMotor.configure(
            Configs.ClimbSubsystem.climbConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

    }

    /*Set the climb motor power in a range of [-1, 1] */
    private void setClimbPower(double power){
        climbMotor.set(power);
    }

    /**
     * Command to run the climb motor UP. When the command is interrupted, e.g. the button is released,
     * the motor will stop.
    */
    public Command climbUpCommand() {
    return this.startEnd(
        () -> this.setClimbPower(ClimbSetpoints.kClimbUp), () -> this.setClimbPower(0.0));
    }
    
    /**
     * Command to run the climb motor DOWN. When the command is interrupted, e.g. the button is released,
     * the motor will stop.
    */
    public Command climbDownCommand() {
        return this.startEnd(
            () -> this.setClimbPower(ClimbSetpoints.kClimbDown), () -> this.setClimbPower(0.0));
        }

}
