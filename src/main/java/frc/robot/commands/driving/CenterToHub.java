package frc.robot.commands.driving;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pi;

public class CenterToHub extends CommandBase {
    private final PIDController pid;
    private final Drivetrain drive;

    public CenterToHub(Drivetrain drive) {
        super();
        this.drive = drive;
        // addRequirements(drive);
        pid = new PIDController(0.75, 0.05, 0.0);
        pid.setSetpoint(320.0);
        pid.setTolerance(20.0);
    }

    @Override
    public void execute() {
        double pidVal = pid.calculate(Pi.getTargetCenterX());
        if (Math.abs(pidVal) > 60) {
            pidVal = Math.signum(pidVal) * 60;
        } else if (Math.abs(pidVal) < 30) {
            pidVal = Math.signum(pidVal) * 30;
        }

        drive.swerveDrive(0.0, 0.0, Math.toRadians(-pidVal), false);

        double positionError = pid.getPositionError();
        SmartDashboard.putNumber("Centering PID error", positionError);
        SmartDashboard.putNumber("Centering PID value", pidVal);
    }

    @Override
    public void end(boolean interrupted) {
        drive.swerveDrive(0.0, 0.0, 0.0, false);
        //System.out.println("CenterToHub end");
    }

    @Override
    public boolean isFinished() {
        // System.out.println("CenterToHub finished");
        return pid.atSetpoint();
    }
}
