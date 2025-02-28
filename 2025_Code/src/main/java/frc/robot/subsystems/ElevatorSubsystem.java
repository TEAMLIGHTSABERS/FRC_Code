package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        kDriverInput,
        kWDriverInput;
    }

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

    /*// Simulation setup and variables
    private DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
    private SparkFlexSim elevatorMotorSim;
    private SparkLimitSwitchSim elevatorLimitSwitchSim;
    private final ElevatorSim m_elevatorSim =
        new ElevatorSim(
            elevatorMotorModel,
            SimulationRobotConstants.kElevatorGearing,
            SimulationRobotConstants.kCarriageMass,
            SimulationRobotConstants.kElevatorDrumRadius,
            SimulationRobotConstants.kMinElevatorHeightMeters,
            SimulationRobotConstants.kMaxElevatorHeightMeters,
            true,
            SimulationRobotConstants.kMinElevatorHeightMeters,
            0.0,
            0.0);

    // Mechanism2d setup for subsystem
    private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
    private final MechanismLigament2d m_elevatorMech2d =
        m_mech2dRoot.append(
            new MechanismLigament2d(
                "Elevator",
                SimulationRobotConstants.kMinElevatorHeightMeters
                    * SimulationRobotConstants.kPixelsPerMeter,
                90));*/

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

        SmartDashboard.setDefaultNumber("Elevator Driver Input", 0);
        SmartDashboard.setDefaultNumber("Wrist Driver Input", 0);
                
        
    }

        /**
         * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
         * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
         * setpoints.
         */
        private void moveToSetpoint() {
            elevatorClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
            wristClosedLoopController.setReference(wristCurrentTarget, ControlType.kMAXMotionPositionControl);
            //wristClosedLoopController.setReference(wristCurrentTarget, ControlType.kPosition);
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
    
        /* Prep for dropping off at level 2 of the reef */
        //public void setSetpointReef2(){
        //    setSetpointCommand(Setpoint.kLevel2);
        //}
        public Command setSetpointReef2(){
            return setSetpointCommand(Setpoint.kLevel2);
        }
    
        /* Prep for dropping off at level 4 of the reef */
        /*public void setSetpointReef4(){
            setSetpointCommand(Setpoint.kLevel4);
        }*/
    
        /* Prep for picking up at the coral feeder station */
        /*public void setSetpointCoralStation(){
            setSetpointCommand(Setpoint.kFeederStation);
        }*/
    
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
                    break;
                case kLevel1:
                    wristCurrentTarget = WristSetpoints.kWLevel1;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
                    intakeDirection = IntakeSetpoints.kForward;
                    break;
                case kLevel2:
                    wristCurrentTarget = WristSetpoints.kWLevel2;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
                    intakeDirection = IntakeSetpoints.kForward;
                    break;
                case kLevel3:
                    wristCurrentTarget = WristSetpoints.kWLevel3;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
                    intakeDirection = IntakeSetpoints.kForward;
                    break;
                case kLevel4:
                    wristCurrentTarget = WristSetpoints.kWLevel4;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
                    intakeDirection = IntakeSetpoints.kForward;
                    break;
                case kDriverInput:            
                    elevatorCurrentTarget = ElevatorSetpoints.kDriverInput;
                    wristCurrentTarget = WristSetpoints.kWDriverInput;
                    break;
                case kWDriverInput:
                    elevatorCurrentTarget = 0;
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
        
    
        @Override
        public void periodic() {
        moveToSetpoint();
        zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

        ElevatorSetpoints.kDriverInput = SmartDashboard.getNumber("Elevator Driver Input", 0);
        WristSetpoints.kWDriverInput = SmartDashboard.getNumber("Wrist Driver Input", 0);
    
        // Display subsystem values
        SmartDashboard.putNumber("Wrist/Target Position", wristCurrentTarget);
        SmartDashboard.putNumber("Wrist/Actual Position", wristEncoder.getPosition());
        SmartDashboard.putNumber("Elevator/Target Position", elevatorCurrentTarget);
        SmartDashboard.putNumber("Elevator/Actual Position", elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Elevator/Actual Velocity", elevatorEncoder.getVelocity());
        SmartDashboard.putNumber("Intake/Applied Output", intakeMotor.getAppliedOutput());
    
        /*// Update mechanism2d
        m_elevatorMech2d.setLength(
            SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
                + SimulationRobotConstants.kPixelsPerMeter
                    * (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                    * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
        m_armMech2d.setAngle(
            180
                - ( // mirror the angles so they display in the correct direction
                Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                    + Units.rotationsToDegrees(
                        armEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
                - 90 // subtract 90 degrees to account for the elevator
            );*/
        }
    
        /** Get the current drawn by each simulation physics model */
        /*public double getSimulationCurrentDraw() {
        return m_elevatorSim.getCurrentDrawAmps() + m_armSim.getCurrentDrawAmps();
        }*/
    
        /*@Override
        public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    
        // Update sim limit switch
        elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);
    
        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);
        m_armSim.update(0.020);
    
        // Iterate the elevator and arm SPARK simulations
        elevatorMotorSim.iterate(
            ((m_elevatorSim.getVelocityMetersPerSecond()
                        / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                    * SimulationRobotConstants.kElevatorGearing)
                * 60.0,
            RobotController.getBatteryVoltage(),
            0.02);
        armMotorSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(
                m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
            RobotController.getBatteryVoltage(),
            0.02);
    
        // SimBattery is updated in Robot.java
        }*/
}
