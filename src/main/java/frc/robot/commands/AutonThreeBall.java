package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

public class AutonThreeBall extends CommandBase {
    private Drivetrain drive;

    public AutonThreeBall(Drivetrain drive) {
        
        this.drive = drive;
        addRequirements(drive);
        drive.currentStep = 0;

    }

    public void initialize() {
        // TODO: what happens if we start on the equivalent red side of the field?
        drive.odometry.resetPosition(new Pose2d(8.586, 6.05, new Rotation2d()), new Rotation2d());
        drive.currentStep++;
    }


    private void stop() {
        drive.drive(0, 0, 0, true);
    }

    public void execute() {
        //TODO decrease time parameter to speed up the robot
        System.out.println(drive.currentStep);
        //8.586, 7.107836, 90 for first ball
        drive.setPosition(8.586, 7.107836, Math.toRadians(90), 2, 1);
        //TODO ingest

        //turn 180
        drive.setPosition(8.696, 7.0007836, Math.toRadians(270), 2, 2);
        

        //TODO shoot

        //drive to next ball: 10.340534, 6.433, -30 degrees
        drive.setPosition(10.340534, 6.433, Math.toRadians(330), 2, 3);
        
        //TODO ingest

        //turn to 220 degrees
        drive.setPosition(10.450534, 6.543, Math.toRadians(220), 2, 4);
        
        
        //TODO shoot

    }

    public boolean isFinished() {
        return drive.currentStep == 5; //change this if any setPosition steps are added in execute()
    }

    public void end() {
        drive.drive(0, 0, 0, true);
    }

}
