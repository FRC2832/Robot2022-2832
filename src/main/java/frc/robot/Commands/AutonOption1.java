package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;

public class AutonOption1 extends CommandBase {
    private Drivetrain drive;
    private Timer timer;
    private double delay = 4.0;

    public AutonOption1(Drivetrain drive) {
        this.drive = drive;
        timer = new Timer();
        timer.start();
        delay = SmartDashboard.getNumber("Shooting delay", 4.0); // TODO: check that drivers can change this value
        addRequirements(drive);
    }

    public void execute() {
        double driveTime = 3;
        if(timer.get() < driveTime) {
            drive.drive(-1.0, 0.0, 0.0, false);
        }
        else {
            drive.drive(0.0, 0.0, 0.0, false);
            if(timer.get() > delay + driveTime) {
                drive.drive(0.0, 0.0, 0.0, false);
                System.out.println("Shooting"); // TODO: actually shoot
            }
        }
    }

}