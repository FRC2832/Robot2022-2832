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
        // addRequirements(drive); // TODO: This is commented out on CenterToHub. Interesting. May be worth investigating if we need it here.
        pid = new PIDController(0.65, 0.0, 0.0);
        pid.setSetpoint(375.0);
        pid.setTolerance(15.0);
    }

    @Override
    public void execute() {
        double pidVal = pid.calculate(Pi.getCargoCenterX());
        if (Math.abs(pidVal) > 60) {
            pidVal = Math.signum(pidVal) * 60;
        } else if (Math.abs(pidVal) < 30) {
            pidVal = Math.signum(pidVal) * 30;
        }

        drive.swerveDrive(0.0, 0.0, Math.toRadians(-pidVal), false);

        SmartDashboard.putNumber("Centering PID error", pid.getPositionError());
        SmartDashboard.putNumber("Centering PID value", pidVal);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.swerveDrive(0.0, 0.0, 0.0, false);
    }
}
