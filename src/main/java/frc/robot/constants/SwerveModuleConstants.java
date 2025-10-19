package frc.robot.constants;

public final class SwerveModuleConstants {
    private SwerveModuleConstants() {}

    public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI;

    public static final int ENCODER_CPR = 1024;
    public static final double WHEEL_DIAMETER_METERS = 0.15;

    // Assumes the encoders are directly mounted on the wheel shafts
    public static final double DRIVE_ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER_METERS * Math.PI) / ENCODER_CPR;

    // Assumes the encoders are on a 1:1 reduction with the module shaft.
    public static final double TURNING_ENCODER_DISTANCE_PER_PULSE = (2 * Math.PI) / ENCODER_CPR;

    public static final double P_MODULE_DRIVE_CONTROLLER = 1;
    public static final double I_MODULE_DRIVE_CONTROLLER = 0;
    public static final double D_MODULE_DRIVE_CONTROLLER = 0;

    public static final double P_MODULE_TURNING_CONTROLLER = 1;
    public static final double I_MODULE_TURNING_CONTROLLER = 0;
    public static final double D_MODULE_TURNING_CONTROLLER = 0;
}
