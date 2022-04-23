package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveStick extends CommandBase {
    private final Drivetrain drive;
    private final XboxController controller;

    public DriveStick(Drivetrain drive, XboxController controller) {
        super();
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        double xSpeed = -Drivetrain.deadbandStick(controller.getLeftY()) * Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        double ySpeed = -Drivetrain.deadbandStick(controller.getLeftX()) * Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        double rot = -Drivetrain.deadbandStick(controller.getRightX()) * Drivetrain.kMaxAngularSpeed;

        // ask the drivetrain to run
        drive.swerveDrive(xSpeed, ySpeed, rot, false);
    }
}
