package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

public class AutoDrive extends CommandBase {
    private final Drivetrain drive;
    private final double xSpeed;
    private final double ySpeed;

    public AutoDrive(Drivetrain drive, double xSpeed, double ySpeed) {
        this.drive = drive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // ask the drivetrain to run
        drive.swerveDrive(xSpeed, ySpeed, 0.0, true);
    }
}
