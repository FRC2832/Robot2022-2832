package frc.robot.CommandGroup;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;

public class Auton1A extends CommandBase {
    private Drivetrain drive;
    // static double startTime = 0.0;
    private Timer timer;
    private double delay = 4.0;

    public Auton1A(Drivetrain drive) {
        this.drive = drive;
        // startTime = Timer.getFPGATimestamp();
        timer = new Timer();
        timer.start();
        delay = SmartDashboard.getNumber("Shooting delay", 4.0);
        // SmartDashboard.putNumber("Shooting delay", 4.0); // TODO: check that drivers can change this value
        addRequirements(drive);
    }

    public void execute() {
        // double elapsedTime = Timer.getFPGATimestamp() - startTime;
        double driveTime = 3;
        if(timer.get() < driveTime) {
            drive.drive(-1.0, 0.0, 0.0, true);
            System.out.println("moving back");
        }
        else {
            drive.drive(0.0, 0.0, 0.0, true);
            System.out.println("stopping");
            if(timer.get() > delay + driveTime) {
                drive.drive(0.0, 0.0, 0.0, true);
                System.out.println("Shooting"); // TODO: actually shoot
            }
        }
    }

}
