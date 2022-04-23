package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Auton1 extends CommandBase {
    private final Drivetrain drive;
    private final Timer timer;

    public Auton1(Drivetrain drive) {
        super();
        timer = new Timer();
        this.drive = drive;
        //TIMER.start();
        addRequirements(drive);
    }


    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // shoots preloaded ball and backs off tarmac
    @Override
    public void execute() {
        double timerVal = timer.get();
        if (timerVal < 3.0) {
            System.out.println("Shooting");
            drive.swerveDrive(0.0, 0.0, 0.0, false);
        } else if (timerVal < 5.0) {
            drive.swerveDrive(-1.0, 0.0, 0.0, false);
        } else {
            drive.swerveDrive(0.0, 0.0, 0.0, false);
            timer.stop();
        }
    }
}
