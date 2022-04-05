package frc.robot.commands;

import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonOption3 extends CommandBase {
    private final Timer timer;
    private final Drivetrain drive;
    private final double delay;

    public AutonOption3(Drivetrain drive) {
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
        delay = SmartDashboard.getNumber("Shooting Delay:", 0.0);
    }

    private void stop() {
        drive.drive(0, 0, 0, true);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double timerVal = timer.get();
        // timer.start();
        if (timerVal < (2 + delay)) {
            System.out.println("Moving Back");
            drive.drive(-1, 0, 0, false);
        } else if (timerVal < 3 + delay) {
            System.out.println("Stopping");
            stop();
        }
        // Pick up ball
        // shoot times two
        else if (timerVal < 6 + delay) {
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