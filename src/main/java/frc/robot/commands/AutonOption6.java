package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

public class AutonOption6 extends CommandBase {
    private final Drivetrain drive;
    private final Timer timer;

    public AutonOption6(Drivetrain drive) {
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
        double timerVal = timer.get();
        if (timerVal < 2) {
            drive.drive(.75, 0, 0, false);
            System.out.println("Getting ball 1");
        } else if (timerVal > 2 && timerVal < 3) {
            stop();
            System.out.println("SHOOTING!!!!");
        }
        // pick up ball and shoot x2
        else if (timerVal > 3 && timerVal < 4) {
            drive.drive(0, 0, -3 * Math.PI / 4, false);
        } else if (timerVal > 4 && timerVal < 7) {
            drive.drive(1, 0, 0, false);
            System.out.println("Getting ball 2");
        } else if (timerVal > 8 && timerVal < 9) {
            drive.drive(0, 0, Math.PI / 2.5, false);
        } else if (timerVal > 9 && timerVal < 12) {
            drive.drive(1.5, 0, 0, false);
        } else if (timerVal > 12 && timerVal < 13) {
            drive.drive(0, 0, Math.PI, false);
        } else if (timerVal > 13 && timerVal < 14) {
            drive.drive(2, 0, 0, false);
        } else {
            stop();
            System.out.println("SHOOTING!!!!");
        }
    }
    // Now we shoot!!!!!
}