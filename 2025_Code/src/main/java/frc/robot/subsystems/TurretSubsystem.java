package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    // Class variables
    private ShuffleboardTab TurretTab;
    private GenericEntry selectZeroEntry;
    //private GenericEntry selectOneEntry;
    private GenericEntry selectTwoEntry;
    private GenericEntry selectThreeEntry;
    private GenericEntry selectFFGainEntry;
    private GenericEntry selectPGainEntry;
    private GenericEntry currSelPosEntry;
    private GenericEntry currWenchPosEntry;

    private static int selectedPosition;
    private static double commandedWenchPosition;
    private static double currentWenchPosition;
    private SparkPIDController elevationPIDCtrl;
    private static int inputDelayCtr;

    private static double kStartDeg, kHighShotDeg, kAmpDeg;
    private static double startDeg, highShotDeg, ampDeg;
    private static double kFFGain, fFGain, saveFFGain;
    private static double kPGain, pGain, savePGain;

    // Class Hardware
    private CANSparkMax elevationMotor;
    private RelativeEncoder elevationRelEncoder;
    private Servo deflServo;

    // Class Constructor
    public TurretSubsystem(){
        // Tunable Constants
        kStartDeg = Constants.Turret.kStartPosition;
        kHighShotDeg = Constants.Turret.kHighShotPosition; //Speaker
        kAmpDeg = Constants.Turret.kAmpPosition;
        kFFGain = Constants.Turret.kVEGainStartUp;
        kPGain = Constants.Turret.kPEGainStartUp;
        saveFFGain = Constants.Turret.kVEGainStartUp;
        savePGain = Constants.Turret.kPEGainStartUp;

        // Initialize operational variables
        selectedPosition = 0; // Hanging Position
        currentWenchPosition = convertSelPosToWench(selectedPosition);
        commandedWenchPosition = currentWenchPosition;
        inputDelayCtr = 0;
    
        // Turret status
        TurretTab = Shuffleboard.getTab("Turret Subsystem");

        selectZeroEntry = TurretTab
        .add("Sel 0 Pos", kStartDeg).getEntry();
        selectTwoEntry = TurretTab
        .add("Sel 2 Pos", kHighShotDeg).getEntry();
        selectThreeEntry = TurretTab
        .add("Sel 3 Pos", kAmpDeg).getEntry();
        selectFFGainEntry = TurretTab
        .add("Sel FF Gain", kFFGain).getEntry();
        selectPGainEntry = TurretTab
        .add("Sel P Gain", kPGain).getEntry();

        currSelPosEntry = TurretTab
        .add("Current Selected Position", kStartDeg).getEntry();
        currWenchPosEntry = TurretTab
        .add("Current Turrent Position", currentWenchPosition).getEntry();

        TurretTab.add("Accept 0th", acceptZeroSetting());
        TurretTab.add("Accept 2nd", acceptTwoSetting());
        TurretTab.add("Accept 3rd", acceptThreeSetting());
        TurretTab.add("Accept FFGain", acceptFFGainSetting());
        TurretTab.add("Accept PGain", acceptPGainSetting());
    
        elevationMotor =
        new CANSparkMax(Constants.Turret.kTurCanId, CANSparkLowLevel.MotorType.kBrushless);

        elevationMotor.setInverted(false );
        elevationMotor.setSmartCurrentLimit((int) kFFGain);
        elevationMotor.setIdleMode(IdleMode.kBrake);

        elevationRelEncoder = elevationMotor.getEncoder();
        elevationRelEncoder.setPositionConversionFactor(22.0); // Degrees/Shaft
        elevationRelEncoder.setVelocityConversionFactor(22.0); // dpm/Shaft
        elevationRelEncoder.setPosition(0.0);


        elevationPIDCtrl = elevationMotor.getPIDController();
        elevationPIDCtrl.setP(kPGain);
        elevationPIDCtrl.setI(Constants.Turret.kIEController);
        elevationPIDCtrl.setD(Constants.Turret.kDEController);
        elevationPIDCtrl.setFF(kFFGain);
        elevationPIDCtrl.setFeedbackDevice(elevationRelEncoder);
        elevationPIDCtrl.setOutputRange(-1, 1); 

        deflServo = new Servo(Constants.Turret.kDeflID);
        deflServo.set(Constants.Turret.kDeflOff);

    }

    /**
    * Method to drive the turret elevation to one of three positions.
    *
    * @param povDegrees  The position indicator from the POV.
    *                         Up is Speaker Position,
    *                         Right is Amp position, and
    *                         Down is Center Stand Positon.
    */
    public void driveWench(Boolean upCommand, Boolean downCommand) {

        if(upCommand){
            advancePOS();
        } else if (downCommand){
            if (inputDelayCtr == Constants.OIConstants.kInputDelayTimedOut){
                reducePOS();
                inputDelayCtr = 0;
            }
        }
    };

    /**
    * Command to move the Turret to a specified Position.
    *
    * @param ChangePosTo  The position indicator from the POV.
    *                         Up is Speaker Position,
    *                         Right is Amp position, and
    *                         Down is Center Stand Positon.
    */
/*     public Command moveTo( (int) requestedPos) {
        Command changePosTo = new Command(){
            int currSelPos =getSelPos();
            while(currSelPos != RequestedPos)
            {
                if(currSelPos < requestedPos){
                    advancePOS();
                }
            }
        }
        if(upCommand){
            advancePOS();
        } else if (downCommand){
            if (inputDelayCtr == Constants.OIConstants.kInputDelayTimedOut){
                reducePOS();
                inputDelayCtr = 0;
            }
        }
    };
*/
    /**
    * Constructs a command for a button that accepts the Gear position (in deg) 
    * for the Zeroth Turret Position.
    *
    * @return The setStart command
    */
    public Command acceptZeroSetting(){
    Command setStart = 
        new Command() {
        @Override
        public void initialize() {
            startDeg = selectZeroEntry.getDouble(kStartDeg); 
            kStartDeg = startDeg;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setStart;
    };

    /**
    * Constructs a command for a button that accepts the Gear position (in deg) 
    * for the 2nd Turret Position.
    *
    * @return The SetHighShot command
    */
    public Command acceptTwoSetting(){
    Command setHighShot = 
        new Command() {
        @Override
        public void initialize() {
            highShotDeg = selectTwoEntry.getDouble(kHighShotDeg); 
            kHighShotDeg = highShotDeg;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setHighShot;
    };

    /**
    * Constructs a command for a button that accepts the Gear position (in deg) 
    * for the 3rd Turret Position.
    *
    * @return The SetAmp command
    */
    public Command acceptThreeSetting(){
    Command setAmp = 
        new Command() {
        @Override
        public void initialize() {
            ampDeg = selectThreeEntry.getDouble(kAmpDeg); 
            kAmpDeg = ampDeg;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setAmp;
    };

    /**
    * Constructs a command for a button that accepts the Current Limit (percentage).
    *
    * @return The SetCurrLmt command
    */
    public Command acceptFFGainSetting(){
    Command setCurrLmt = 
        new Command() {
        @Override
        public void initialize() {
            fFGain = selectFFGainEntry.getDouble(kFFGain); 
            kFFGain = fFGain;
            elevationPIDCtrl.setFF(kFFGain);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setCurrLmt;
    };
    
    /**
    * Constructs a command for a button that accepts the P Gain.
    *
    * @return The SetPGain command
    */
    public Command acceptPGainSetting(){
    Command setCurrLmt = 
        new Command() {
        @Override
        public void initialize() {
            pGain = selectPGainEntry.getDouble(kPGain); 
            kPGain = pGain;
            elevationPIDCtrl.setP(kPGain);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setCurrLmt;
    };

    @Override
    public void periodic() { // this method will be called once per scheduler run
        // set the launcher motor powers based on whether the launcher is on or not
        double posError;

        if (inputDelayCtr < Constants.OIConstants.kInputDelayTimedOut){
            inputDelayCtr++;
        }

        commandedWenchPosition = convertSelPosToWench(selectedPosition);
        currentWenchPosition = elevationRelEncoder.getPosition();

        posError = commandedWenchPosition - currentWenchPosition;
        elevationPIDCtrl.setReference(posError, ControlType.kPosition);

        // Add Launcher Power Wheel Rates to the Launcher Subsystem Tab on Shuffleboard.
        currSelPosEntry.setInteger(selectedPosition);
        currWenchPosEntry.setDouble(currentWenchPosition);
        SmartDashboard.putNumber("Commanded Wench Position", commandedWenchPosition);
        SmartDashboard.putNumber("Current Wench Position", currentWenchPosition);

    }

    public int getSelPosition (){
        return (selectedPosition);
    }

    private double convertSelPosToWench (int _SelectedPos) {
        double wenchPosition;

        switch (_SelectedPos) {
        case 0: // Start Position
            wenchPosition = kStartDeg;    // Rotations
            break;
        /*case 1: // Speaker Position
            wenchPosition = kLongShotDeg;   // Rotations
            break;*/
        case 2: // Speaker Position
            wenchPosition = kHighShotDeg;   // Rotations
            break;
        case 3: // Amp Position
        default:
            wenchPosition = kAmpDeg;  // Rotations
            break;
        };

        return wenchPosition;
    }

    public void advancePOS(){
        if (selectedPosition < Constants.Turret.kAmpID){
            selectedPosition++;
            if(selectedPosition == 1){
                selectedPosition++;
            }

            switch (selectedPosition) {
                case Constants.Turret.kHighShotID:
                    savePGain = selectPGainEntry.getDouble(kPGain);
                    saveFFGain = selectFFGainEntry.getDouble(kFFGain);

                    kPGain = Constants.Turret.kPEGainHighShotUp;
                    kFFGain = Constants.Turret.kVEGainHighShotUp; 
                    elevationPIDCtrl.setP(kPGain);
                    elevationPIDCtrl.setFF(kFFGain);

                    selectPGainEntry.setDouble(kPGain);
                    selectFFGainEntry.setDouble(kFFGain);
                    break;
            
                case Constants.Turret.kAmpID:

                    deflServo.set(Constants.Turret.kDeflON);

                    savePGain = selectPGainEntry.getDouble(kPGain);
                    saveFFGain = selectFFGainEntry.getDouble(kFFGain);

                    kPGain = Constants.Turret.kPEGainAmpUp;
                    kFFGain = Constants.Turret.kVEGainAmpUp; 
                    elevationPIDCtrl.setP(kPGain);
                    elevationPIDCtrl.setFF(kFFGain);

                    selectPGainEntry.setDouble(kPGain);
                    selectFFGainEntry.setDouble(kFFGain);
                    break;
            
                default:

                    kPGain = savePGain;
                    kFFGain = saveFFGain; 
                    elevationPIDCtrl.setP(kPGain);
                    elevationPIDCtrl.setFF(kFFGain);

                    selectPGainEntry.setDouble(kPGain);
                    selectFFGainEntry.setDouble(kFFGain);
                    break;
            }

            currSelPosEntry.setInteger(selectedPosition);
            SmartDashboard.putNumber("Selected Position", selectedPosition);
        }
    }

    public void reducePOS(){
        if (selectedPosition > Constants.Turret.kStartID){
            selectedPosition--;
            if(selectedPosition == 1){
                selectedPosition--;
            }

            switch (selectedPosition) {
                case Constants.Turret.kHighShotID:

                    deflServo.set(Constants.Turret.kDeflOff);

                    savePGain = selectPGainEntry.getDouble(kPGain);
                    saveFFGain = selectFFGainEntry.getDouble(kFFGain);

                    kPGain = Constants.Turret.kPEGainHighShotDown;
                    kFFGain = Constants.Turret.kVEGainHighShotDown; 
                    elevationPIDCtrl.setP(kPGain);
                    elevationPIDCtrl.setFF(kFFGain);

                    selectPGainEntry.setDouble(kPGain);
                    selectFFGainEntry.setDouble(kFFGain);

                    break;
            
                case Constants.Turret.kStartID:
                    savePGain = selectPGainEntry.getDouble(kPGain);
                    saveFFGain = selectFFGainEntry.getDouble(kFFGain);

                    kPGain = Constants.Turret.kPEGainStartDown;
                    kFFGain = Constants.Turret.kVEGainStartDown; 
                    elevationPIDCtrl.setP(kPGain);
                    elevationPIDCtrl.setFF(kFFGain);

                    selectPGainEntry.setDouble(kPGain);
                    selectFFGainEntry.setDouble(kFFGain);

                    break;
            
                default:
                    
                    kPGain = savePGain;
                    kFFGain = saveFFGain; 
                    elevationPIDCtrl.setP(kPGain);
                    elevationPIDCtrl.setFF(kFFGain);

                    selectPGainEntry.setDouble(kPGain);
                    selectFFGainEntry.setDouble(kFFGain);
                    break;
            }

            currSelPosEntry.setInteger(selectedPosition);
            SmartDashboard.putNumber("Selected Position", selectedPosition);
        }
    }
}
