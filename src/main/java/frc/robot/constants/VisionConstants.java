package frc.robot.constants;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class VisionConstants {
    private VisionConstants() {
    }

    public static final String CAMERA_NAME = "Simulation Cam";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));

    public static final SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties()
            .setCalibration(960, 720, Rotation2d.fromDegrees(90))
            .setCalibError(0.35, 0.10)
            .setFPS(0)
            .setAvgLatencyMs(0)
            .setLatencyStdDevMs(0);

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
}
