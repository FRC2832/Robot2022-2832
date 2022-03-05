package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter;

public class DashboardShoot extends CommandBase {
    private Shooter shooter;

    public DashboardShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", 2000);
    }
    
    @Override
    public void execute() {
        double rpm = SmartDashboard.getNumber("Target RPM", 2000);
        shooter.setShooterRpm(rpm);
    }
}