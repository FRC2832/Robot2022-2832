package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;

public class AutonOption1 extends CommandBase {
    private Drivetrain drive;
    private Timer timer;
    private double delay = 0.0;
    private static final double DRIVE_TIME = 1.5;

    public AutonOption1(Drivetrain drive) {
        this.drive = drive;
        timer = new Timer();
        timer.start();
        delay = SmartDashboard.getNumber("Shooting delay", 0.0);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double timerVal = timer.get();
        if (timerVal < DRIVE_TIME) {
            drive.drive(-1.0, 0.0, 0.0, false);
        } else {
            drive.drive(0.0, 0.0, 0.0, false);
            if (timerVal > delay + DRIVE_TIME) {
                drive.drive(0.0, 0.0, 0.0, false);
                System.out.println("Shooting"); // TODO: actually shoot
            }
        }
    }

}