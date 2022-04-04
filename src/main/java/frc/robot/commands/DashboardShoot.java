package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter;
import edu.wpi.first.wpilibj.XboxController;

public class DashboardShoot extends CommandBase {
    private Shooter shooter;
    private XboxController driverController;

    public DashboardShoot(Shooter shooter, XboxController driverController) {
        this.shooter = shooter;
        this.driverController = driverController;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", 2000);
    }

    @Override
    public void execute() {
        if (driverController.getYButton()) {
            double rpm = SmartDashboard.getNumber("Target RPM", 2000);
            shooter.setShooterRpm(rpm);
        }
    }
}