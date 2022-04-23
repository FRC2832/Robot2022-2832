package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Option2Auton extends CommandBase {
    private final Drivetrain drive;

    public Option2Auton(Drivetrain drive) {
        super();
        this.drive = drive;
        addRequirements(drive);
        drive.resetCurrentStep();
    }

    @Override
    public void initialize() {
        // TODO: what happens if we start on the equivalent red side of the field?
        //drive.odometry.resetPosition(new Pose2d(6.5, 5.0, new Rotation2d()), new Rotation2d());
        drive.incrementCurrentStep();
    }

    @Override
    public void execute() {
        System.out.println("it is executing \n" + drive.getCurrentStep());

        /*drive.setPosition(5.06, 5.96, Math.toRadians(135), 1, 1); // to blueBalls[0]
        // ingest- CommandScheduler.getInstance().schedule(new Ingestor());?

        // shoot - CommandScheduler.getInstance().schedule(new AutoShoot());?
        drive.setPosition(5.99, 6.95, Math.toRadians(50), 1, 2); // to redBalls[7]
        // ingest
        drive.setPosition(4.65, 6.84, Math.toRadians(190), 1, 3); // to hangar
        // shoot lightly into hangar - DashboardShoot but with a value of 1000 rpm and
        // 39 degrees hood angle
        drive.setPosition(6, 7, 0, 1, 4); // towards center line*/
    }

    @Override
    public void end(boolean interrupted) {
        drive.swerveDrive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return drive.getCurrentStep() == 5; // change this if any setPosition steps are added in execute()
    }

}