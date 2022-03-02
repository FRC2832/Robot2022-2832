package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;

public class Auton1 extends CommandBase {
    private Drivetrain drive;
    private Timer timer;

    public Auton1(Drivetrain drive) {
        this.drive = drive;
        timer = new Timer();
        timer.start();
        addRequirements(drive);
    }

    // shoots preloaded ball and backs off tarmac
    public void execute() {
        if(timer.get() < 3) {
            System.out.println("Shooting");
            drive.drive(0.0, 0.0, 0.0, false);
        } else if(timer.get() >= 3 && timer.get() < 5) {
            drive.drive(-1.0, 0.0, 0.0, false);
        } else {
            drive.drive(0.0, 0.0, 0.0, false);
        }
    }
}
