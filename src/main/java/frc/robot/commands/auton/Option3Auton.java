package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Option3Auton extends CommandBase {
    private final Timer timer;
    private final Drivetrain drive;
    private final double delay;

    public Option3Auton(Drivetrain drive) {
        super();
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
        delay = SmartDashboard.getNumber("Shooting Delay:", 0.0);
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
            drive.swerveDrive(-1, 0, 0, false);
        } else if (timerVal < 3 + delay) {
            System.out.println("Stopping");
            stop();
        }
        // Pick up ball
        // shoot times two
        else if (timerVal < 6 + delay) {
            System.out.println("Moving Left");
            drive.swerveDrive(0, 1, 0, false);
        } else {
            System.out.println("Stopping");
            stop();
        }
        // timer.stop();
        // timer.reset();

    }

    private void stop() {
        drive.swerveDrive(0, 0, 0, true);
    }

}