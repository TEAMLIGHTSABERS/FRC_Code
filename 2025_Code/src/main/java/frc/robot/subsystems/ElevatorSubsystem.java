package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.ElevatorSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.ElevatorSubsystemConstants.WristSetpoints;

public class ElevatorSubsystem extends SubsystemBase {
    /** Subsystem-wide setpoints */
    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4,
        kAlgae1,
        kAlgae2,
        kDriverInput;
    }

    private final Timer timer = new Timer();

    /*Elevator Motor Initialization - Lead motor and follower */
    private SparkMax l_elevatorMotor =
        new SparkMax(ElevatorSubsystemConstants.kElevatorLeadCanId, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController =
        l_elevatorMotor.getClosedLoopController();

    private RelativeEncoder elevatorEncoder = l_elevatorMotor.getEncoder();

    private SparkMax f_elevatorMotor =
        new SparkMax(ElevatorSubsystemConstants.kElevatorFollowCanId, MotorType.kBrushless);

    /*Wrist Motor Initialization*/
    private SparkMax wristMotor =
        new SparkMax(ElevatorSubsystemConstants.kWristMotorCanId, MotorType.kBrushless);

    private SparkClosedLoopController wristClosedLoopController =
        wristMotor.getClosedLoopController();

    private RelativeEncoder wristEncoder = wristMotor.getEncoder();
    
    /*Intake Motor Initialization */
    private SparkMax intakeMotor =
        new SparkMax(ElevatorSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

    // Member variables for subsystem state management
    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;
    private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
    private double wristCurrentTarget;
    private double intakeDirection = IntakeSetpoints.kReverse;
    private String scorePos = "Initialized";

    public ElevatorSubsystem() {
        /*
            * Apply the appropriate configurations to the SPARKs.
            *
            * kResetSafeParameters is used to get the SPARK to a known state. This
            * is useful in case the SPARK is replaced.
            *
            * kPersistParameters is used to ensure the configuration is not lost when
            * the SPARK loses power. This is useful for power cycles that may occur
            * mid-operation.
            */
        l_elevatorMotor.configure(
            Configs.ElevatorSubsystem.l_elevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        f_elevatorMotor.configure(
            Configs.ElevatorSubsystem.f_elevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        wristMotor.configure(
            Configs.ElevatorSubsystem.wristConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
        intakeMotor.configure(
            Configs.ElevatorSubsystem.intakeConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);
       
        // Zero elevator encoders on initialization
        elevatorEncoder.setPosition(0);
        wristEncoder.setPosition(0);

        SmartDashboard.setDefaultString("Scoring Position", scorePos);

        SmartDashboard.setDefaultNumber("Elevator Driver Input", 0);
        SmartDashboard.setDefaultNumber("Coral Station Adjust", WristSetpoints.kWFeederStation);
        SmartDashboard.setDefaultNumber("Wrist Driver Input", 0);
        SmartDashboard.setDefaultBoolean("Driver Input Enter", false);

        SmartDashboard.setDefaultNumber("Wrist P-Value", WristSetpoints.wristP);
        SmartDashboard.setDefaultNumber("Wrist D-Value", WristSetpoints.wristD);
        SmartDashboard.setDefaultNumber("Elevator P-Value", ElevatorSetpoints.kElevatorP);
        SmartDashboard.setDefaultNumber("Elevator D-Value", ElevatorSetpoints.kElevatorD);
        SmartDashboard.setDefaultBoolean("PID Enter", false);
    }

        /**
         * Drive the wrist and elevator motors to their respective setpoints. This will use MAXMotion
         * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
         * setpoints.
         */
        private void moveToSetpoint() {         
            wristClosedLoopController.setReference(wristCurrentTarget, ControlType.kMAXMotionPositionControl);
            elevatorClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
        }
    
        /** Zero the elevator encoder when the limit switch is pressed. */
        private void zeroElevatorOnLimitSwitch() {
        if (!wasResetByLimit && l_elevatorMotor.getReverseLimitSwitch().isPressed()) {
            // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
            // prevent constant zeroing while pressed
            elevatorEncoder.setPosition(0);
            wasResetByLimit = true;
        } else if (!l_elevatorMotor.getReverseLimitSwitch().isPressed()) {
            wasResetByLimit = false;
        }
        }
    
        /** Zero the elevator encoders when the user button is pressed on the roboRIO. */
        private void zeroOnUserButton() {
        if (!wasResetByButton && RobotController.getUserButton()) {
            // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
            // constant zeroing while pressed
            wasResetByButton = true;
            wristEncoder.setPosition(0);
            elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
        }
/*
        /* Set the intake power in the range of [-1,1]. */
        private void setIntakePower(double power){
            intakeMotor.set(power);
        }

        /* Set the intake power to 0 to stop the intake. */
        private void stopIntake(){
            intakeMotor.set(0);
        }
        
        /**
         * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
         * positions for the given setpoint.
         */
        public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(
            () -> {
                switch (setpoint) {
                case kFeederStation:
                    wristCurrentTarget = WristSetpoints.kWFeederStation;
                    elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
                    intakeDirection = IntakeSetpoints.kReverse;
                    scorePos = "Coral Station";
                    break;
                case kLevel1:
                    wristCurrentTarget = WristSetpoints.kWLevel1;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
                    intakeDirection = IntakeSetpoints.kForward;
                    scorePos = "Level 1";
                    break;
                case kLevel2:
                    wristCurrentTarget = WristSetpoints.kWLevel2;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
                    intakeDirection = IntakeSetpoints.kForward;
                    scorePos = "Level 2";
                    break;
                case kLevel3:
                    wristCurrentTarget = WristSetpoints.kWLevel3;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
                    intakeDirection = IntakeSetpoints.kForward;
                    scorePos = "Level 3";
                    break;
                case kLevel4:
                    wristCurrentTarget = WristSetpoints.kWLevel4;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
                    intakeDirection = IntakeSetpoints.kL4Forward;
                    scorePos = "Level 4";
                    break;
                case kAlgae1:
                    wristCurrentTarget = WristSetpoints.kWAlgae;
                    elevatorCurrentTarget = ElevatorSetpoints.kAlgae1;
                    intakeDirection = IntakeSetpoints.kAlgae;
                    scorePos = "Algae Level 1";
                    break;
                case kAlgae2:
                    wristCurrentTarget = WristSetpoints.kWAlgae;
                    elevatorCurrentTarget = ElevatorSetpoints.kAlgae2;
                    intakeDirection = IntakeSetpoints.kAlgae;
                    scorePos = "Algae Level 2";
                    break;
                case kDriverInput:            
                    elevatorCurrentTarget = ElevatorSetpoints.kDriverInput;
                    wristCurrentTarget = WristSetpoints.kWDriverInput;
                    break;
                }
            });

        }
     
        /**
         * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
         * the motor will stop.
         */
        public Command forwardIntakeCommand() {
        return this.startEnd(
            () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));
        }
    
        /**
         * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
         * released, the motor will stop.
         */
        public Command reverseIntakeCommand() {
        return this.startEnd(
            () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
        }

        public Command runIntakeCommand(){
            return this.startEnd(
            () -> this.setIntakePower(intakeDirection), () -> this.setIntakePower(0.0));
        }
        
        public Command autoIntakeCommand(double power, double time){
            return runOnce(() -> {
                timer.reset();
                timer.start();
                setIntakePower(power);
            }).andThen(run(() -> {
                if (timer.get() >= time) {
                    stopIntake();
                }
            }).until(() -> timer.get() >= time))
              .finallyDo((interrupted) -> {
                  stopIntake();
                  timer.stop();
              });
        }
    
        @Override
        public void periodic() {
        moveToSetpoint();
        zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

        if(SmartDashboard.getBoolean("Driver Input Enter", true)){
        ElevatorSetpoints.kDriverInput = SmartDashboard.getNumber("Elevator Driver Input", 0);
        WristSetpoints.kWDriverInput = SmartDashboard.getNumber("Wrist Driver Input", 0);
        WristSetpoints.kWFeederStation = SmartDashboard.getNumber("Coral Station Adjust", WristSetpoints.kWFeederStation);
        }

        if(SmartDashboard.getBoolean("PID Enter", true)){
        WristSetpoints.wristP = SmartDashboard.getNumber("Wrist P-Value", WristSetpoints.wristP);
        WristSetpoints.wristD = SmartDashboard.getNumber("Wrist D-Value", WristSetpoints.wristD);
        ElevatorSetpoints.kElevatorP = SmartDashboard.getNumber("Elevator P-Value", ElevatorSetpoints.kElevatorP);
        ElevatorSetpoints.kElevatorD = SmartDashboard.getNumber("Elevator D-Value", ElevatorSetpoints.kElevatorD);
        }
    
        // Display subsystem values
        SmartDashboard.putNumber("Wrist/Target Position", wristCurrentTarget);
        SmartDashboard.putNumber("Wrist/Actual Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Wrist/Actual Velocity", wristEncoder.getVelocity());
        SmartDashboard.putNumber("Wrist/P-Value", WristSetpoints.wristP);
        SmartDashboard.putNumber("Wrist/Coral Station Pos", WristSetpoints.kWFeederStation);

        SmartDashboard.putNumber("Elevator/Target Position", elevatorCurrentTarget);
        SmartDashboard.putNumber("Elevator/Actual Position", elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Elevator/Actual Velocity", elevatorEncoder.getVelocity());
        SmartDashboard.putNumber("Elevator/P-Value", ElevatorSetpoints.kElevatorP);
        SmartDashboard.putNumber("Elevator/D-Value", ElevatorSetpoints.kElevatorD);
        
        SmartDashboard.putNumber("Intake/Applied Output", intakeMotor.getAppliedOutput());
    }
}
