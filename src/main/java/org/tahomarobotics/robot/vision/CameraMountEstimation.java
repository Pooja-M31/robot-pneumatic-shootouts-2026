package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.function.Consumer;

import static org.tahomarobotics.robot.vision.VisionConstants.FIELD_LAYOUT;

public class CameraMountEstimation {

    private final Consumer<AprilTagCamera.EstimatedRobotPose> estimationCallback =
        Chassis.getInstance()::processVisionUpdate;

    private boolean disableCameraPoseEstimate = false;

    public CameraMountEstimation() {

        SmartDashboard.putData(
            "Chassis Pose!", Commands.runOnce(() -> {
                setRobotPose(Chassis.getInstance().getPose());

            })
        );

        SmartDashboard.putBoolean("CameraEst", disableCameraPoseEstimate);

        setRobotPose(Chassis.getInstance().getPose());
    }

    public Consumer<AprilTagCamera.EstimatedRobotPose> chainConsumer(Consumer<AprilTagCamera.EstimatedRobotPose> original) {
        return estimatedRobotPose -> {
            calculate(estimatedRobotPose);
            if (!disableCameraPoseEstimate) {
                original.accept(estimatedRobotPose);
            }
        };
    }


    public void setRobotPose(Pose2d robotPose2d) {
        SmartDashboard.putNumber("pose-X", Units.metersToInches(robotPose2d.getX()));
        SmartDashboard.putNumber("pose-Y", Units.metersToInches(robotPose2d.getY()));
        SmartDashboard.putNumber("pose-Heading", robotPose2d.getRotation().getDegrees());
        System.out.println("pose update to chassis");
    }

    public void calculate(AprilTagCamera.EstimatedRobotPose estimatedRobotPose) {

        disableCameraPoseEstimate = SmartDashboard.getBoolean("CameraEst", false);

        double x = Units.inchesToMeters(SmartDashboard.getNumber("pose-X", 0));
        double y = Units.inchesToMeters(SmartDashboard.getNumber("pose-Y", 0));
        double h = Units.degreesToRadians(SmartDashboard.getNumber("pose-Heading", 0));

        Pose2d robotPose2d = new Pose2d(x, y, new Rotation2d(h));

        for (PhotonTrackedTarget target : estimatedRobotPose.targets) {

            Transform3d cameraToTargetTranspose = target.getBestCameraToTarget();

            Pose3d targetPose = FIELD_LAYOUT.getTagPose(target.getFiducialId()).orElseThrow();

            Pose3d cameraPose = targetPose.plus(cameraToTargetTranspose.inverse());

            Transform3d cameraToRobotTransform = cameraPose.minus(new Pose3d(robotPose2d));

            Translation3d t = cameraToRobotTransform.getTranslation();
            Rotation3d r = cameraToRobotTransform.getRotation();
            // TODO undo this change
            SmartDashboard.putNumberArray(
                estimatedRobotPose.cameraName + "-Camera2Robot", new double[]{
                    Units.metersToInches(t.getX()),
                    Units.metersToInches(t.getY()),
                    Units.metersToInches(t.getZ()),
                    Units.radiansToDegrees(r.getX()),
                    Units.radiansToDegrees(r.getY()),
                    Units.radiansToDegrees(r.getZ())}
            );

            System.out.println(estimatedRobotPose.cameraName + " (" + target.fiducialId + ")---> " + cameraToRobotTransform);

        }
    }
}
