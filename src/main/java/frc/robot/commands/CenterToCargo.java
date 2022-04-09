package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Pi;

public class CenterToCargo extends CommandBase {
    private Drivetrain drive;
    private PIDController pid;

    public CenterToCargo(Drivetrain drive) {
        this.drive = drive;
        addRequirements(drive);
        pid = new PIDController(0.35, 0, 0); // values from tyros last year were 0.35, 0.05, 0.8
        pid.setSetpoint(320);
        pid.setTolerance(10); // tolerance of 10 pixels
    }

    @Override
    public void execute() {
        double pidVal = pid.calculate(Pi.getCargoCenterX());
        if (Math.abs(pidVal) > 60) {
            pidVal = Math.signum(pidVal) * 60;
        }

        drive.swerveDrive(0, 0, Math.toRadians(-pidVal), false);

        SmartDashboard.putNumber("Centering PID error", pid.getPositionError());
        SmartDashboard.putNumber("Centering PID value", pidVal);
    }

    @Override
    public void end(boolean interrupted) {
        drive.swerveDrive(0, 0, 0, false);
    }
}
