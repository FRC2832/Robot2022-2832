package frc.robot.commands;

import frc.robot.Drivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonOption2 extends CommandBase {
    // private Timer timer;
    private Drivetrain drive;
    // private double delay = 4.0;

    public AutonOption2(Drivetrain drive) {
        // timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
        // delay = SmartDashboard.getNumber("Shooting delay", 0.0);
        // timer.start();
        drive.currentStep = 0;
    }

    public void execute() {

        if(drive.currentStep == 0) {
            drive.odometry.resetPosition(new Pose2d(6.5, 5.0, new Rotation2d()), new Rotation2d());
            drive.currentStep++;
        }

        // TODO: what happens if we start on the equivalent red side of the field?
        drive.setPosition(5.06, 5.96, Math.toRadians(135), 0.5, 1); // to blueBalls[0]

        // ingest
        // shoot - CommandScheduler.getInstance().schedule(new AutoShoot());?

        drive.setPosition(5.99, 6.95, Math.toRadians(50), .5, 2); // to redBalls[7]

        // ingest

        drive.setPosition(4.65, 6.84, Math.toRadians(190), .5, 3); // to hangar

        // shoot lightly into hangar - DashboardShoot but with a value of 1000 rpm and 39 degrees hood angle

        drive.setPosition(6, 7, 0, .5, 4); //towards center line
    }

    public boolean isFinished() {
        return drive.currentStep == 5; //change this if any setPosition steps are added in execute()
    }

    public void end() {
        drive.drive(0, 0, 0, true);
    }

}