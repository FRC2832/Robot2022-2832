package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class DashboardShoot extends ShootCommand {
    private final XboxController driverController;

    public DashboardShoot(Shooter shooter, Ingestor ingestor, XboxController driverController) {
        super(shooter, ingestor, 2000, shooter.getHoodAngle());
        this.driverController = driverController;
        SmartDashboard.putNumber("Target RPM", 2000);
    }

    @Override
    public void execute() {
        if (driverController.getYButton()) {
            targetRpm = SmartDashboard.getNumber("Target RPM", 2000);
            shooter.setShooterRpm(targetRpm);
        }
    }
}