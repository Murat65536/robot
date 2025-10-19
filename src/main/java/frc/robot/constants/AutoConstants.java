package frc.robot.constants;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class AutoConstants {
    private AutoConstants() {
    }

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double P_X_CONTROLLER = 1;
    public static final double P_Y_CONTROLLER = 1;
    public static final double P_THETA_CONTROLLER = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final Optional<RobotConfig> ROBOT_CONFIG = loadRobotConfig();

    public static Optional<RobotConfig> loadRobotConfig() {
        try {
            return Optional.of(RobotConfig.fromGUISettings());
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        return Optional.empty();
    }
}
