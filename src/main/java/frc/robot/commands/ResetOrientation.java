package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Command to run one-shot to reset our orientation.
 */
public class ResetOrientation extends CommandBase {
    private final Drivetrain drive;

    public ResetOrientation(Drivetrain drive) {
        super();
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
