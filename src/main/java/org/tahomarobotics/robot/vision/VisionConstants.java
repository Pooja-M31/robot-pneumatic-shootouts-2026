package org.tahomarobotics.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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

//    // Loading custom apriltag layout (for use with WPICal)
//    static {
//        try {
//            String fieldLayoutPath = RobotBase.isReal() ?
//                Filesystem.getOperatingDirectory() + "/deploy/2025-reefscape.json" :
//                Filesystem.getOperatingDirectory() + "\\src\\main\\deploy\\2025-reefscape.json";
//            FIELD_LAYOUT = new AprilTagFieldLayout(fieldLayoutPath);
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
//    }

    // Cameras

    // These values are from CAD and may not reflect real life position

    public final static CameraConfiguration COLLECTOR_SIDE = new CameraConfiguration(
        "Collector Side",
        new Transform3d(
            new Translation3d(0.13522837777536, 0.2286, 0.75736194111635 + 0.041),
            new Rotation3d(0, Units.degreesToRadians(28.36904629327858), 0)
        ),
        StandardDeviationScaling.DEFAULT
    );

    public final static CameraConfiguration SCORING_SIDE = new CameraConfiguration(
        "Scoring Side",
        new Transform3d(
            new Translation3d(0.13522837777536 - 0.32, 0.2286, 0.78215000726482),
            new Rotation3d(0, Units.degreesToRadians(40), Units.degreesToRadians(180))
        ),
        StandardDeviationScaling.DEFAULT
    );

    // Standard Deviations

    public static final Vector<N3> BASE_MULTI_TAG_STD_DEV = VecBuilder.fill(0.25, 0.25, Double.POSITIVE_INFINITY);
    public static final Vector<N3> BASE_SINGLE_TAG_STD_DEV = VecBuilder.fill(0.25, 0.25, Double.POSITIVE_INFINITY);

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
