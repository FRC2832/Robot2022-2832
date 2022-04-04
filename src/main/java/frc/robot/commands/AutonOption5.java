package frc.robot.commands;

import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonOption5 extends CommandBase {
    private Timer timer;
    private Drivetrain drive;

    public AutonOption5(Drivetrain drive) {
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
        timer.start();
    }

    private void stop() {
        drive.drive(0, 0, 0, true);
    }

    @Override
    public void execute() {
        // timer.start();
        double timerVal = timer.get();
        if (timerVal < 2) {
            System.out.println("Moving Back");
            drive.drive(-1, 0, 0, false);
        } else if (timerVal > 2 && timerVal < 3) {
            System.out.println("Stopping");
            stop();
        }
        // INGEST BALL
        // SHOOT TWO BALLS
        else if (timerVal > 3 && timerVal < 7) {
            System.out.println("Moving Back Towards Terminal");
            drive.drive(-1, 0, 0, false);
        } else if (timerVal > 7 && timerVal < 8) {
            System.out.println("Stopping");
            stop();
        }
        // INGEST BALL
        // INGEST BALL FED FROM HUMAN PLAYER
        else if (timerVal > 8 && timerVal < 11) {
            System.out.println("Moving towards Central Hub");
            drive.drive(1, 0, 0, false);
        } else if (timerVal > 11 && timerVal < 12) {
            System.out.println("Stopping");
            stop();
        }
        // SHOOT TWO BALLS
        else {
            System.out.println("Stopping");
            stop();
        }
        // timer.stop();
        // timer.reset();
    }
}