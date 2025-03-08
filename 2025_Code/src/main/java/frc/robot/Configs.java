package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ElevatorSubsystemConstants.WristSetpoints;
import frc.robot.Constants.ElevatorSubsystemConstants;

public final class Configs {
    
    public static final class ClimbSubsystem{
        public static final SparkMaxConfig climbConfig = new SparkMaxConfig();

        static {
                // Configure basic settings of the climb motor
                climbConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        }
    }    

    public static final class ElevatorSubsystem {
        public static final SparkMaxConfig l_elevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig f_elevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig wristConfig = new SparkMaxConfig();
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

        
        static {
                // Configure basic settings of the wrist motor
                wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false).voltageCompensation(12);
                                             
                /*
                 * Configure the closed loop controller with MaxMotion. We want to make sure we set the
                 * feedback sensor as the primary encoder.
                 */
                
                wristConfig
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for position control
                        .p(WristSetpoints.wristP)
                        .d(WristSetpoints.wristD)
                        .outputRange(-1, 1)
                        .maxMotion
                        // Set MAXMotion parameters for position control
                        .maxVelocity(420)
                        .maxAcceleration(600)
                        .allowedClosedLoopError(0.05);

                /*wristConfig
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for position control
                        .p(WristSetpoints.wristP)
                        .d(WristSetpoints.wristD)
                        .outputRange(-.25, .25);*/


                                                     
                // Configure basic settings of the elevator motors
                l_elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12);
                f_elevatorConfig.idleMode(IdleMode.kBrake).follow(ElevatorSubsystemConstants.kElevatorLeadCanId, true).inverted(true).smartCurrentLimit(50).voltageCompensation(12);
         
                /*
                * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
                * will prevent any actuation of the elevator in the reverse direction if the limit switch is
                * pressed.
                */
                l_elevatorConfig
                        .limitSwitch
                        .reverseLimitSwitchEnabled(true)
                        .reverseLimitSwitchType(Type.kNormallyOpen);

                
                /*
                * Configure the closed loop controller. We want to make sure we set the
                * feedback sensor as the primary encoder.
                */
                l_elevatorConfig 
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for position control
                        .p(0.01)
                        //velocityFF(0.0085)
                        .outputRange(-1, 1)
                        .maxMotion
                        // Set MAXMotion parameters for position control
                        .maxVelocity(4200)
                        .maxAcceleration(6000)
                        .allowedClosedLoopError(0.25);

                intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
                }
        }

    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        
        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}