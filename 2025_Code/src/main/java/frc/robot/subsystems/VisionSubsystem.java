package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase{

private final String limelightName = "limelight";
private final DriveSubsystem m_robotDrive = new DriveSubsystem();

private static final double POSITION_TOLERANCE = 0.05; // Meters
private static final double ANGLE_TOLERANCE = 2.0; // Degrees
private Translation2d offset = new Translation2d(0.5, 0.5); // Default offset from AprilTag

public Pose2d getAprilTagPose() {
    double[] botpose = LimelightHelpers.getBotPose_wpiBlue(limelightName);
    if (botpose.length < 6) return null;

    double x = botpose[0]; // X Position in meters
    double y = botpose[1]; // Y Position in meters
    double rotation = botpose[5]; // Rotation in degrees

    return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
}

public void setOffset(Translation2d newOffset) {
    this.offset = newOffset;
}

public void moveToAprilTag() {
    Pose2d detectedPose = getAprilTagPose();
    if (detectedPose == null) return;

    // Calculate target position with offset
    Translation2d targetTranslation = detectedPose.getTranslation().plus(offset);
    Pose2d targetPose = new Pose2d(targetTranslation, detectedPose.getRotation());

    // Compute speeds using HolonomicDriveController
    var speeds = controller.calculate(detectedPose, targetPose, 0, targetPose.getRotation());

    swerve.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
}

}
