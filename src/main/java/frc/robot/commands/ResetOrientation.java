package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

/**
 * Command to run one-shot to reset our orientation.
 */
public class ResetOrientation extends CommandBase {
    Drivetrain drive;

    public ResetOrientation(Drivetrain drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.resetRobot();
    }

    @Override
    public boolean isFinished() {
        // always return true so it only runs 1 loop.
        return true;
    }
}
