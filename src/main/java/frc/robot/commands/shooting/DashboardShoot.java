package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class DashboardShoot extends ShootCommand {
    private final XboxController driverController;

    public DashboardShoot(Shooter shooter, XboxController driverController) {
        super(shooter, 2000.0);
        this.driverController = driverController;
        SmartDashboard.putNumber("Target RPM", 2000);
    }

    @Override
    public void execute() {
        if (driverController.getYButton()) {
            targetRpm = SmartDashboard.getNumber("Target RPM", 2000);
            shooter.setShooterRpm(targetRpm);
        } // TODO: Do we need an else statement here?
    }
}