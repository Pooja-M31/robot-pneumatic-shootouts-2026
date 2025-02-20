package org.tahomarobotics.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.simulation.SimCameraProperties;

/** Constants for the {@link Vision} subsystem. */
public class VisionConstants {
    // AprilTag Field Layout

    // TODO: Convert this to be a mapping from field name (or some other identifier) to a layout so we can support
    //  multiple fields.
    /**
     * The AprilTag field layout for whatever field we are on. See {@link AprilTagCamera}'s JavaDoc for more information
     * on how to calculate this properly.
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Cameras

    public final static CameraConfiguration ELEVATOR_SWERVE = new CameraConfiguration(
        "Elevator Swerve",
        new Transform3d(
            new Translation3d(-0.25, 0.2971, 0.20),
            new Rotation3d(new Quaternion(-0.08672828390630943, 0.06988593908540369, -0.01731288468135895, 0.9936269039799738))
        ),
        StandardDeviationScaling.DEFAULT
    );

    public final static CameraConfiguration CLIMBER_SWERVE = new CameraConfiguration(
        "Climber Swerve",
        new Transform3d(
            new Translation3d(-0.25, -0.2971, 0.20),
            new Rotation3d(new Quaternion(0.17379550957598747, 0.07964208982705674, 0.019307103639845842, 0.9813661366321145))
        ),
        StandardDeviationScaling.DEFAULT
    );

    // Standard Deviations

    public static final Vector<N3> BASE_MULTI_TAG_STD_DEV = VecBuilder.fill(0.25, 0.25, 0.5);
    public static final Vector<N3> BASE_SINGLE_TAG_STD_DEV = VecBuilder.fill(0.75, 0.75, 1);

    public static final double AMBIGUITY_THRESHOLD = 0.03;

    // Constraint Punishment

    public static final boolean HEADING_FREE = false;
    public static final double GYRO_ERROR_SCALING_FACTOR = 500.0;

    // Simulation

    public static final SimCameraProperties simOV9782Properties = new SimCameraProperties() {{
        setCalibration(
            1280,
            720,
            MatBuilder.fill(
                Nat.N3(), Nat.N3(),
                895.0681882368845, 0.0, 711.9376583910714,
                0.0, 896.6336103968874, 333.5574273453275,
                0.0, 0.0, 1.0
            ),
            VecBuilder.fill(
                0.011040036794979738,
                0.025690451227094003,
                0.0012670750613393597,
                -1.079822477748635E-4,
                -0.05583469833028936,
                5.147188640387755E-4,
                6.085269216455457E-4,
                0.003908226961469329
            )
        );
        setCalibError(0.35, 0.10);
        setFPS(30);
        setAvgLatencyMs(30);
        setLatencyStdDevMs(10);
    }};

    // Camera Configuration

    @Logged
    public record CameraConfiguration(String name, Transform3d transform, StandardDeviationScaling stdDevScaling) {}

    @Logged(strategy = Logged.Strategy.OPT_IN)
    public interface StandardDeviationScaling {
        Vector<N3> scaleStandardDeviations(Vector<N3> stdDevs, double distance, int targetCount);

        StandardDeviationScaling DEFAULT = (stdDevs, distance, targetCount) -> stdDevs;
    }
}
