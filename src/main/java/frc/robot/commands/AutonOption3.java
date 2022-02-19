package frc.robot.commands;

import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonOption3 extends CommandBase {
    private Timer timer;
    private Drivetrain drive;
    private double delay = 4.0;

    public AutonOption3(Drivetrain drive) {
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
        delay = SmartDashboard.getNumber("Shooting Delay:", 0.0);
        timer.start();
    }

    private void stop() {
        drive.drive(0, 0, 0, true);
    }

    public void execute() {

        // timer.start();
        if (timer.get() < (2 + delay)) {
            System.out.println("Moving Back");
            drive.drive(-1, 0, 0, false);
        } else if (timer.get() > 2 + delay && timer.get() < 3 + delay) {
            System.out.println("Stopping");
            stop();
        }
        // Pick up ball
        // shoot times two
        else if (timer.get() > 3 + delay && timer.get() < 6 + delay) {
            System.out.println("Moving Left");
            drive.drive(0, 1, 0, false);
        } else {
            System.out.println("Stopping");
            stop();
        }
        // timer.stop();
        // timer.reset();

    }

}