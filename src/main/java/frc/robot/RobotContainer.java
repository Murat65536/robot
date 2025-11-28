package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.GamepieceLauncher;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class RobotContainer {
    private final SwerveDrive drivetrain = new SwerveDrive();
    private final GamepieceLauncher gamepieceLauncher = new GamepieceLauncher();
    private final Vision vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getSimPose);
    private final XboxController controller = new XboxController(0);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        AutoBuilder.configure(
                drivetrain::getPose,
                (pose) -> resetPose(pose),
                drivetrain::getChassisSpeeds,
                (speeds, feedForwards) -> drivetrain.setChassisSpeeds(speeds, true, true),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                AutoConstants.ROBOT_CONFIG,
                () -> {
                    final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drivetrain);
        // NamedCommands.registerCommand("");
        autoChooser = AutoBuilder.buildAutoChooser();
        configureDashboard();
        configureButtonBindings();
        configureDefaultCommands();
    }

    private void configureDashboard() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new RunCommand(() -> {
            double forward = -controller.getLeftY() * DriveConstants.MAX_LINEAR_SPEED;
            double strafe = -controller.getLeftX() * DriveConstants.MAX_LINEAR_SPEED;
            double turn = -controller.getRightX() * DriveConstants.MAX_ANGULAR_SPEED;
            drivetrain.drive(forward, strafe, turn);
        }, drivetrain));
    }

    public void updateBatteryVoltage() {
        // Calculate battery voltage sag due to current draw
        double batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw());

        // Using max(0.1, voltage) here isn't a *physically correct* solution,
        // but it avoids problems with battery voltage measuring 0.
        RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose, true);
        vision.resetSimPose(pose);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
