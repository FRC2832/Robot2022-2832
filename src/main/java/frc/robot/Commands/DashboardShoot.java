package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DashboardShoot extends CommandBase{
    @Override
    public void execute() {
        double rpm = SmartDashboard.getNumber("Target RPM", 1000);
    }
}
