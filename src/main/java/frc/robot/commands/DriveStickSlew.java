package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

public class DriveStickSlew extends CommandBase {
    private Drivetrain drive;
    private XboxController controller;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    public DriveStickSlew(Drivetrain drive, XboxController controller) {
        this.drive = drive;
        this.controller = controller;
        addRequirements(drive);
    }

    public void initialize() {

    }

    public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final double xSpeed = xSpeedLimiter.calculate(drive.deadbandStick(controller.getLeftY()))
                * frc.robot.Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final double ySpeed = ySpeedLimiter.calculate(drive.deadbandStick(controller.getLeftX()))
                * frc.robot.Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final double rot = -rotLimiter.calculate(drive.deadbandStick(controller.getRightX()))
                * frc.robot.Drivetrain.kMaxAngularSpeed;

        // ask the drivetrain to run
        drive.drive(xSpeed, ySpeed, rot, false);
    }
}
