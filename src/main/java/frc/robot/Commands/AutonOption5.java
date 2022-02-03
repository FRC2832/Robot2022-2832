package frc.robot.Commands;

import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public void execute() {

        // timer.start();
        if (timer.get() < (2)) {
            System.out.println("Moving Back");
            drive.drive(-1, 0, 0, false);
        } 
        else if (timer.get() > 2 && timer.get() < 3) {
            System.out.println("Stopping");
            stop();
        }
        // INGEST BALL
        // SHOOT TWO BALLS
        else if (timer.get() > 3 && timer.get() < 7) {
            System.out.println("Moving Back Towards Terminal");
            drive.drive(-1, 0, 0, false);
        } 
        else if (timer.get() > 7 && timer.get() < 8){
            System.out.println("Stopping");
            stop();
        }
        // INGEST BALL
        // INGEST BALL FED FROM HUMAN PLAYER
        else if (timer.get() > 8 && timer.get() < 11){
            System.out.println("Moving towards Central Hub");
            drive.drive(1, 0, 0, false);
        }
        else if (timer.get() > 11 && timer.get() < 12){
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