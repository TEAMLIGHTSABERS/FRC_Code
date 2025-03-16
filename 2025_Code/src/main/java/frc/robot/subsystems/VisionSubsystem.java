package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase{

    private final String limelightName = "limelight";
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

    private static final double TX_TOLERANCE = 1.0; // Degrees tolerance for left/right alignment
    private static final double TY_TOLERANCE = 1.0; // Degrees tolerance for distance adjustment
    private static final double ROTATION_TOLERANCE = 2.0; // Degrees tolerance for rotation

    //private Translation2d offset = new Translation2d(0.5, 0.5); // Default offset from AprilTag

        public VisionSubsystem(){
            xController.setTolerance(TX_TOLERANCE);
            yController.setTolerance(TY_TOLERANCE);
            thetaController.setTolerance(ROTATION_TOLERANCE);    
        }

        /*public void setOffset(Translation2d newOffset) {
            this.offset = newOffset;
        }*/

        public void moveToAprilTag(Translation2d offset) {
            double tx = LimelightHelpers.getTX(limelightName); // Left/Right error
            double ty = LimelightHelpers.getTY(limelightName); // Up/Down error

            if (Double.isNaN(tx) || Double.isNaN(ty)) return; // If no AprilTag is detected, don't move

            // Convert TX and TY to a distance-based movement by applying the offset
            double targetX = offset.getX(); // Move backward by offset X
            double targetY = offset.getY();  // Move right by offset Y

            double xSpeed = xController.calculate(tx, targetX); // Move left/right
            double ySpeed = yController.calculate(ty, targetY); // Move forward/backward
            double rotationSpeed = thetaController.calculate(tx, 0); // Rotate to face the tag

            m_robotDrive.drive(xSpeed, ySpeed, rotationSpeed, false);
        }

        public boolean isAtTarget() {
            return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
        }

        public void stop() {
            m_robotDrive.stop();
        }

}