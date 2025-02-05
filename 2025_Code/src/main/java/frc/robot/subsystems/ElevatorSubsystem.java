package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.ElevatorSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.SimulationRobotConstants;




public class ElevatorSubsystem extends SubsystemBase {
    /** Subsystem-wide setpoints */
    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
    }

    private SparkMax l_elevatorMotor =
        new SparkMax(ElevatorSubsystemConstants.kElevatorLeadCanId, MotorType.kBrushless);

    private SparkClosedLoopController elevatorClosedLoopController =
        l_elevatorMotor.getClosedLoopController();

    private RelativeEncoder elevatorEncoder = l_elevatorMotor.getEncoder();

    private SparkMax f_elevatorMotor =
        new SparkMax(ElevatorSubsystemConstants.kElevatorFollowCanId, MotorType.kBrushless);

    // Member variables for subsystem state management
    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;
    private double elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;

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
    
        /*// Display mechanism2d
        SmartDashboard.putData("Elevator Subsystem", m_mech2d);*/
    
        // Zero elevator encoders on initialization
        elevatorEncoder.setPosition(0);
    
        /*// Initialize simulation values
        elevatorMotorSim = new SparkFlexSim(elevatorMotor, elevatorMotorModel);
        elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
        armMotorSim = new SparkMaxSim(armMotor, armMotorModel);*/
        }
    
        /**
         * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
         * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
         * setpoints.
         */
        private void moveToSetpoint() {
        //armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        elevatorClosedLoopController.setReference(
            elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
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
            //armEncoder.setPosition(0);
            elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
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
                    //armCurrentTarget = ArmSetpoints.kFeederStation;
                    elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
                    break;
                case kLevel1:
                    //armCurrentTarget = ArmSetpoints.kLevel1;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
                    break;
                case kLevel2:
                    //armCurrentTarget = ArmSetpoints.kLevel2;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
                    break;
                case kLevel3:
                    //armCurrentTarget = ArmSetpoints.kLevel3;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
                    break;
                case kLevel4:
                    //armCurrentTarget = ArmSetpoints.kLevel4;
                    elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
                    break;
                }
            });
        }
    
        
        /**
         * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
         * the motor will stop.
         */
        /*public Command runIntakeCommand() {
        return this.startEnd(
            () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));
        }*/
    
        /**
         * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
         * released, the motor will stop.
         */
        /*public Command reverseIntakeCommand() {
        return this.startEnd(
            () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
        }*/
    
        @Override
        public void periodic() {
        moveToSetpoint();
        zeroElevatorOnLimitSwitch();
        zeroOnUserButton();
    
        // Display subsystem values
        //SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
       // SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
        SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
        SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
        //SmartDashboard.putNumber("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
    
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
