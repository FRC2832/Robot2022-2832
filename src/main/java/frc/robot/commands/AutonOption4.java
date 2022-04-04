package frc.robot.commands;

import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonOption4 extends CommandBase {
    private Timer timer;
    private Drivetrain drive;

    public AutonOption4(Drivetrain drive) {
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
        timer.start();
        //boolean stopMove = false;
    }

    private void stop() {
        drive.drive(0, 0, 0, false);
    }

    public void execute() {
        // timer.start();
        if (timer.get() < 2) { // simulation endlessly repeating this if statement, doesn't continue to next
            System.out.println("Moving Off Tarmac");
            drive.drive(-1, 0, 0, false);
        }

        else if (timer.get() > 2 && timer.get() < 3) { // change end time to account for below functions
            System.out.println("Stopping");
            stop();
        }

        // INGEST BALL
        // AUTO-AIM TO UPPER HUB
        // SHOOT TWICE (RESET SHOOTER WHEEL SPEED BETWEEN SHOTS FOR ACCURACY)

        /*else if (timer.get() > 3 && timer.get() < 6) {// Change start time to account for above functions
            System.out.println("Moving Left");
            drive.drive(0, 1, 0, false);
        }

        else if (timer.get() > 6 && timer.get() < 7) {
            System.out.println("Stopping");
            stop();
        }

        else if (timer.get() > 7 && timer.get() < 8) {// Change end time to account for below functions
            System.out.println("Moving Forward");
            drive.drive(1, 0, 0, false);
        }*/

        // Combining the three if statements above this into one to streamline the process
        else if (timer.get() > 3 && timer.get() < 7) {
            System.out.println("Moving diagonally towards second ball");
            drive.drive(0.3, 1, 0, false);
        }
        

        else if (timer.get() > 7 && timer.get() < 8) {
            System.out.println("Stopping");
            stop();
        }

        else if (timer.get() > 8 && timer.get() < 9) {
            System.out.println("Turning towards Upper Hub");
            drive.drive(0, 0, -(Math.PI/2), false);
        }

        else if (timer.get() > 9 && timer.get() < 10) {
            System.out.println("Turning to face Ingestor to Terminal");
            drive.drive(0, 0, -(Math.PI/4), false);
        }

        // INGEST BALL
        // AUTO-AIM TO UPPER HUB
        // SHOOT BALL

        else if (timer.get() > 10 && timer.get() < 15) {
            System.out.println("Moving Backwards Towards Terminal");
            drive.drive(-1, 0, 0, false);
        }

        else {
            System.out.println("Stopping");
            stop();
            timer.stop();
        }
        // timer.reset();

    }

}