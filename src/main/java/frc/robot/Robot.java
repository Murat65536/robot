// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.GamepieceLauncher;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class Robot extends TimedRobot {
    private SwerveDrive drivetrain;
    private Vision vision;

    private GamepieceLauncher gpLauncher;

    private XboxController controller;

    @Override
    public void robotInit() {
        drivetrain = new SwerveDrive();
        vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getSimPose);

        controller = new XboxController(0);

        gpLauncher = new GamepieceLauncher();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        drivetrain.log();
    }

    @Override
    public void disabledPeriodic() {
        drivetrain.stop();
    }

    @Override
    public void teleopInit() {
        resetPose();
    }

    @Override
    public void teleopPeriodic() {
        // Calculate drivetrain commands from Joystick values
        double forward = -controller.getLeftY() * DriveConstants.MAX_LINEAR_SPEED;
        double strafe = -controller.getLeftX() * DriveConstants.MAX_LINEAR_SPEED;
        double turn = -controller.getRightX() * DriveConstants.MAX_ANGULAR_SPEED;

        // Command drivetrain motors based on target speeds
        drivetrain.drive(forward, strafe, turn);

        // Calculate whether the gamepiece launcher runs based on our global pose
        // estimate.
        Pose2d curPose = drivetrain.getPose();
        boolean shouldRun = curPose.getY() > 2.0 && curPose.getX() < 4.0; // Close enough to blue speaker
        gpLauncher.setRunning(shouldRun);
    }

    @Override
    public void simulationPeriodic() {
        Field2d debugField = vision.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(drivetrain.getPose());
        debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

        // Update gamepiece launcher simulation
        gpLauncher.simulationPeriodic();

        // Calculate battery voltage sag due to current draw
        double batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw());

        // Using max(0.1, voltage) here isn't a *physically correct* solution,
        // but it avoids problems with battery voltage measuring 0.
        RoboRioSim.setVInVoltage(Math.max(0.1, batteryVoltage));
    }

    public void resetPose() {
        // Example Only - startPose should be derived from some assumption
        // of where your robot was placed on the field.
        // The first pose in an autonomous path is often a good choice.
        Pose2d startPose = new Pose2d(1, 1, new Rotation2d());
        drivetrain.resetPose(startPose, true);
        vision.resetSimPose(startPose);
    }
}
