package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Option4Auton extends CommandBase {
    private final Timer timer;
    private final Drivetrain drive;
    private boolean isDone;

    public Option4Auton(Drivetrain drive) {
        super();
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
        // boolean stopMove = false;
    }

    @Override
    public void initialize() {
        isDone = false;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // timer.start();
        double timerVal = timer.get();
        if (timerVal < 2) { // simulation endlessly repeating this if statement, doesn't continue to next
            System.out.println("Moving Off Tarmac");
            drive.swerveDrive(-1, 0, 0, false);
        } else if (timerVal < 3) { // change end time to account for below functions
            System.out.println("Stopping");
            stop();
        }

        // INGEST BALL
        // AUTO-AIM TO UPPER HUB
        // SHOOT TWICE (RESET SHOOTER WHEEL SPEED BETWEEN SHOTS FOR ACCURACY)

        /*
         * else if (timerVal > 3 && timerVal < 6) {// Change start time to account
         * for above functions
         * System.out.println("Moving Left");
         * drive.drive(0, 1, 0, false);
         * }
         *
         * else if (timerVal > 6 && timerVal < 7) {
         * System.out.println("Stopping");
         * stop();
         * }
         *
         * else if (timerVal > 7 && timerVal < 8) {// Change end time to account
         * for below functions
         * System.out.println("Moving Forward");
         * drive.drive(1, 0, 0, false);
         * }
         */

        // Combining the three if statements above this into one to streamline the
        // process
        else if (timerVal < 7) {
            System.out.println("Moving diagonally towards second ball");
            drive.swerveDrive(0.3, 1, 0, false);
        } else if (timerVal < 8) {
            System.out.println("Stopping");
            stop();
        } else if (timerVal < 9) {
            System.out.println("Turning towards Upper Hub");
            drive.swerveDrive(0, 0, -(Math.PI / 2), false);
        } else if (timerVal < 10) {
            System.out.println("Turning to face Ingestor to Terminal");
            drive.swerveDrive(0, 0, -(Math.PI / 4), false);
        }
        // INGEST BALL
        // AUTO-AIM TO UPPER HUB
        // SHOOT BALL

        else if (timerVal < 15) {
            System.out.println("Moving Backwards Towards Terminal");
            drive.swerveDrive(-1, 0, 0, false);
        } else {
            System.out.println("Stopping");
            stop();
            isDone = true;
            timer.stop();
        }
        // timer.reset();
    }

    private void stop() {
        drive.swerveDrive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}