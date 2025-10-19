package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamepieceLauncher extends SubsystemBase {
    private final PWMSparkMax motor;

    private final double LAUNCH_SPEED_RPM = 2500;
    private double curDesSpd;
    private double curMotorCmd = 0.0;

    public GamepieceLauncher() {
        motor = new PWMSparkMax(8);
        simulationInit();
    }

    public void setRunning(boolean shouldRun) {
        curDesSpd = shouldRun ? LAUNCH_SPEED_RPM : 0.0;
    }

    @Override
    public void periodic() {
        double maxRPM =
                Units.radiansPerSecondToRotationsPerMinute(DCMotor.getFalcon500(1).freeSpeedRadPerSec);
        curMotorCmd = curDesSpd / maxRPM;
        curMotorCmd = MathUtil.clamp(curMotorCmd, 0.0, 1.0);
        motor.set(curMotorCmd);

        SmartDashboard.putNumber("GPLauncher Des Spd (RPM)", curDesSpd);
    }

    // -- SIMULATION SUPPORT
    private DCMotor motorSim;
    private FlywheelSim launcherSim;
    private final double flywheelMoiKgM2 = 0.002;
    private final double flywheelGearRatio = 1.0;

    private void simulationInit() {
        motorSim = DCMotor.getFalcon500(1);
        launcherSim =
                new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(motorSim, flywheelMoiKgM2, flywheelGearRatio),
                        motorSim);
    }

    @Override
    public void simulationPeriodic() {
        launcherSim.setInputVoltage(curMotorCmd * RobotController.getBatteryVoltage());
        launcherSim.update(0.02);
        var spd = launcherSim.getAngularVelocityRPM();
        SmartDashboard.putNumber("GPLauncher Act Spd (RPM)", spd);
    }
}
