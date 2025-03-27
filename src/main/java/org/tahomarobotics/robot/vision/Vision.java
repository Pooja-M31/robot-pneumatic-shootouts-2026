/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.tahomarobotics.robot.auto.AutonomousConstants;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.persistent.CalibrationData;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * A subsystem to consolidate vision updates from multiple AprilTag cameras. Currently, only PhotonVision cameras
 * are supported but if the Limelight 4 is evaluated to be significantly better, then support will be added.
 */
public class Vision extends SubsystemIF implements AutoCloseable {
    private static final Vision INSTANCE = new Vision();

    private static CalibrationData<double[][]> cameraConfigurations;

    // State

    private final Consumer<AprilTagCamera.EstimatedRobotPose> estimationCallback =
        CameraMountEstimation.stream(Chassis.getInstance()::processVisionUpdate);

    // Camera

    public final AprilTagCamera elevatorSwerve =
        new AprilTagCamera(
            VisionConstants.ELEVATOR_SWERVE, VisionConstants.simOV9782Properties,
            estimationCallback
        );

    public final AprilTagCamera climberSwerve =
        new AprilTagCamera(
            VisionConstants.CLIMBER_SWERVE, VisionConstants.simOV9782Properties,
            estimationCallback
        );

    private final Map<String, AprilTagCamera> aprilTagCameras =
        Stream.of(elevatorSwerve, climberSwerve)
              .collect(Collectors.toMap(
                  AprilTagCamera::getName,
                  Function.identity()
              ));

    // LimeLight

    private final NetworkTable LL4 = NetworkTableInstance.getDefault().getTable("limelight");
    private final StringEntry detectorClass = LL4.getStringTopic("tdclass").getEntry("");
    private final IntegerEntry visible = LL4.getIntegerTopic("tv").getEntry(0);
    private final DoubleEntry tx = LL4.getDoubleTopic("tx").getEntry(0);
    private final DoubleEntry ty = LL4.getDoubleTopic("ty").getEntry(0);
    private final DoubleEntry t = LL4.getDoubleTopic("timestamp_LIMELIGHT_publish").getEntry(0);

    // Initialization

    private Vision() {}

    public static Vision getInstance() {
        return INSTANCE;
    }

    // Periodic

    @Override
    public void periodic() {
        getCoralPosition();
    }


    // Setters

    public Optional<Translation2d> getCoralPosition() {
        // Check if we see a coral
        if (!detectorClass.get().equalsIgnoreCase("coral") || visible.get() == 0) {
            return Optional.empty();
        }

        // Get the timestamp and location of the coral in the camera's view
        double tx = this.tx.get();
        double ty = this.ty.get();

        // Get the location of the coral relative to the camera
        double distanceFromCameraToCoral = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.LIME_LIGHT.transform().getZ(),
            Units.inchesToMeters(2),
            VisionConstants.LIME_LIGHT.transform().getRotation().getY(),
            Units.degreesToRadians(ty)
        );
        Logger.recordOutput("Vision/Distance to Coral", distanceFromCameraToCoral);

        if (distanceFromCameraToCoral > VisionConstants.MAX_CORAL_DISTANCE) {
            return Optional.empty();
        }

        Translation2d cameraToCoral = PhotonUtils.estimateCameraToTargetTranslation(
            distanceFromCameraToCoral,
            Rotation2d.fromDegrees(-tx)
        );
        Logger.recordOutput("Vision/Camera To Coral", new Pose3d(new Translation3d(cameraToCoral), new Rotation3d()));

        // Transform to be robot to coral
        Translation2d robotToCoral = cameraToCoral.minus(VisionConstants.LIME_LIGHT.transform().getTranslation().toTranslation2d());
        Logger.recordOutput("Vision/Robot To Coral", new Pose3d(new Translation3d(robotToCoral), new Rotation3d()));

        // Get the chassis pose at the timestamp and transform to be field-to-coral
        Translation2d fieldToCoral = Chassis.getInstance().getPose().plus(new Transform2d(robotToCoral, Rotation2d.k180deg).inverse()).getTranslation();
        Logger.recordOutput("Vision/Coral Position", new Pose3d(new Translation3d(fieldToCoral), new Rotation3d()));

        // Filter out all coral outside the field
        if (!AutonomousConstants.isCoralInField(fieldToCoral)) {
            return Optional.empty();
        }

        return Optional.of(fieldToCoral);
    }

    public void isolate(int tag) {
        org.tinylog.Logger.info("Isolating on tag: " + tag);

        aprilTagCameras.values().forEach(c -> c.isolate(tag));
    }

    public void globalize() {
        org.tinylog.Logger.info("Globalized!");

        aprilTagCameras.values().forEach(AprilTagCamera::globalize);
    }

    public void updateCameraConfigurations() {
        org.tinylog.Logger.info("Starting camera configuration update.");
        climberSwerve.updateConfiguration();
        elevatorSwerve.updateConfiguration();
        org.tinylog.Logger.info("Finished camera configuration update.");
    }

    public void saveCurrentCameraConfigurations() {
        climberSwerve.saveConfiguration();
        elevatorSwerve.saveConfiguration();
        org.tinylog.Logger.info("Saved camera mount positions.");
    }

    // Simulation

    private final VisionSystemSim visionSim = new VisionSystemSim("main");

    @Override
    public void onSimulationInit() {
        visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
        aprilTagCameras.values().forEach(camera -> camera.addCameraToSimulation(visionSim));
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(Chassis.getInstance().getPose());
    }

    // Auto Closeable

    @Override
    public void close() {
        aprilTagCameras.values().forEach(AprilTagCamera::close);
    }
}
