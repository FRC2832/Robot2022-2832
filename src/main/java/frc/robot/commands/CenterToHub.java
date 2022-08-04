package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Pi;

public class CenterToHub extends CommandBase {
    private PIDController pid;
    private Drivetrain drive;
    private int goodCounts;

    public CenterToHub(Drivetrain drive) {
        this.drive = drive;
        // addRequirements(drive);
        pid = new PIDController(0.75, 0.05, 0.0);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(320.0);
        pid.setTolerance(20.0);
        goodCounts = 0;
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

        if(pid.atSetpoint()) {
            goodCounts++;
        } else {
            goodCounts = Math.max(goodCounts--,0);
        }

        SmartDashboard.putNumber("Centering PID error", pid.getPositionError());
        SmartDashboard.putNumber("Centering PID value", pidVal);
        SmartDashboard.putNumber("Centering PID counts", goodCounts);
    }

    @Override
    public boolean isFinished() {
        // System.out.println("CenterToHub finished");
        return goodCounts > 10;
    }

    @Override
    public void end(boolean interrupted) {
        drive.swerveDrive(0.0, 0.0, 0.0, false);
        //System.out.println("CenterToHub end"); // TODO: Comment this out before comp!
    }
}
