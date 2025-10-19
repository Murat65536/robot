package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    private DriveConstants() {
    }

    // Physical properties
    public static final double ROBOT_WIDTH = Units.inchesToMeters(25 + 3.25 * 2);
    public static final double ROBOT_LENGTH = Units.inchesToMeters(25 + 3.25 * 2);
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.5);
    public static final double MAX_ANGULAR_SPEED = Units.rotationsToRadians(2);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 6.75; // 6.75:1 SDS MK4 L2 ratio
    public static final double STEER_GEAR_RATIO = 12.8; // 12.8:1

    public static final double DRIVE_DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / 1024 / DRIVE_GEAR_RATIO;
    public static final double STEER_RAD_PER_PULSE = 2 * Math.PI / 1024;

    public enum ModuleConstants {
        FrontLeft(1, 0, 0, 1, 1, 2, 3, 0, AutoConstants.ROBOT_CONFIG.moduleLocations[0]),
        FrontRight(2, 2, 4, 5, 3, 6, 7, 0, AutoConstants.ROBOT_CONFIG.moduleLocations[1]),
        BackLeft(3, 4, 8, 9, 5, 10, 11, 0, AutoConstants.ROBOT_CONFIG.moduleLocations[2]),
        BackRight(4, 6, 12, 13, 7, 14, 15, 0, AutoConstants.ROBOT_CONFIG.moduleLocations[3]);

        public final int moduleNum;
        public final int driveMotorID;
        public final int driveEncoderA;
        public final int driveEncoderB;
        public final int steerMotorID;
        public final int steerEncoderA;
        public final int steerEncoderB;
        public final double angleOffset;
        public final Translation2d centerOffset;

        private ModuleConstants(
                int moduleNum,
                int driveMotorID,
                int driveEncoderA,
                int driveEncoderB,
                int steerMotorID,
                int steerEncoderA,
                int steerEncoderB,
                double angleOffset,
                Translation2d centerOffset) {
            this.moduleNum = moduleNum;
            this.driveMotorID = driveMotorID;
            this.driveEncoderA = driveEncoderA;
            this.driveEncoderB = driveEncoderB;
            this.steerMotorID = steerMotorID;
            this.steerEncoderA = steerEncoderA;
            this.steerEncoderB = steerEncoderB;
            this.angleOffset = angleOffset;
            this.centerOffset = centerOffset;
        }
    }

    // Feedforward
    // Linear drive feed forward
    public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward( // real
            0.25, // Voltage to break static friction
            2.5, // Volts per meter per second
            0.3 // Volts per meter per second squared
    );
    // Steer feed forward
    public static final SimpleMotorFeedforward STEER_FF = new SimpleMotorFeedforward( // real
            0.5, // Voltage to break static friction
            0.25, // Volts per radian per second
            0.01 // Volts per radian per second squared
    );

    // PID
    public static final double DRIVE_KP = 1;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;

    public static final double STEER_KP = 20;
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0.25;
}
