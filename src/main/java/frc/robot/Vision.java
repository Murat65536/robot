package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM);;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    private final Supplier<Pose2d> poseSupplier;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    /**
     * @param estConsumer Lambda that will accept a pose estimate and pass it to
     *                    your desired {@link
     *                    edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public Vision(EstimateConsumer estConsumer, Supplier<Pose2d> poseSupplier) {
        this.estConsumer = estConsumer;
        this.poseSupplier = poseSupplier;
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, VisionConstants.SIM_CAMERA_PROPERTIES);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAM);

            cameraSim.enableDrawWireframe(true);
        }
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEst;
        for (PhotonPipelineResult change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est -> getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        Matrix<N3, N1> estStdDevs = getEstimationStdDevs();

                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;

        } else {
            // Pose present. Start running Heuristic
            Matrix<N3, N1> estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (PhotonTrackedTarget tgt : targets) {
                Optional<Pose3d> tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation

    @Override
    public void simulationPeriodic() {
        visionSim.update(poseSupplier.get());
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) {
            return null;
        }
        return visionSim.getDebugField();
    }

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
