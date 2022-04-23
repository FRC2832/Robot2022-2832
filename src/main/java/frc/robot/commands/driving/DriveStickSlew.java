package frc.robot.commands.driving;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveStickSlew extends CommandBase {
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private static final SlewRateLimiter X_SPEED_LIMITER = new SlewRateLimiter(3.0);
    private static final SlewRateLimiter Y_SPEED_LIMITER = new SlewRateLimiter(3.0);
    private static final SlewRateLimiter ROT_LIMITER = new SlewRateLimiter(3.0);
    private final Drivetrain drive;
    private final XboxController controller;

    public DriveStickSlew(Drivetrain drive, XboxController controller) {
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
        // no longer inverted because the robot was 'facing' the wrong way with
        // field-relative off
        final double xSpeed =
                X_SPEED_LIMITER.calculate(Drivetrain.deadbandStick(controller.getLeftY())) * Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        // no longer inverted because the robot was 'facing' the wrong way with
        // field-relative off
        final double ySpeed =
                Y_SPEED_LIMITER.calculate(Drivetrain.deadbandStick(controller.getLeftX())) * Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        // no longer inverting this because it was turning the wrong way
        final double rot = ROT_LIMITER.calculate(Drivetrain.deadbandStick(controller.getRightX())) *
                           Drivetrain.kMaxAngularSpeed;

        // ask the drivetrain to run
        drive.swerveDrive(xSpeed, ySpeed, rot, false);
    }
}
