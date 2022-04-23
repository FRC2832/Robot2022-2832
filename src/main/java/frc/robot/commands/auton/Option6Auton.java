package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Option6Auton extends CommandBase {
    private final Drivetrain drive;
    private final Timer timer;

    public Option6Auton(Drivetrain drive) {
        super();
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double timerVal = timer.get();
        if (timerVal < 2) {
            drive.swerveDrive(0.75, 0, 0, false);
            System.out.println("Getting ball 1");
        } else if (timerVal > 2 && timerVal < 3) {
            stop();
            System.out.println("SHOOTING!!!!");
        }
        // pick up ball and shoot x2
        else if (timerVal > 3 && timerVal < 4) {
            drive.swerveDrive(0, 0, -3 * Math.PI / 4, false);
        } else if (timerVal > 4 && timerVal < 7) {
            drive.swerveDrive(1, 0, 0, false);
            System.out.println("Getting ball 2");
        } else if (timerVal > 8 && timerVal < 9) {
            drive.swerveDrive(0, 0, Math.PI / 2.5, false);
        } else if (timerVal > 9 && timerVal < 12) {
            drive.swerveDrive(1.5, 0, 0, false);
        } else if (timerVal > 12 && timerVal < 13) {
            drive.swerveDrive(0, 0, Math.PI, false);
        } else if (timerVal > 13 && timerVal < 14) {
            drive.swerveDrive(2, 0, 0, false);
        } else {
            stop();
            System.out.println("SHOOTING!!!!");
        }
    }

    private void stop() {
        drive.swerveDrive(0, 0, 0, true);
    }
    // Now we shoot!!!!!
}