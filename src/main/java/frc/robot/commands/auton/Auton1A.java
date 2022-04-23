package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Auton1A extends CommandBase {
    private final Drivetrain drive;
    private final Timer timer;
    private final double delay;

    public Auton1A(Drivetrain drive) {
        super();
        timer = new Timer();
        this.drive = drive;
        delay = SmartDashboard.getNumber("Shooting delay", 0.0);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // back up, wait for delay, shoot
    @Override
    public void execute() {
        double driveTime = 3.0;
        double timerVal = timer.get();
        if (timerVal < driveTime) {
            drive.swerveDrive(-1.0, 0.0, 0.0, false);
        } else {
            drive.swerveDrive(0.0, 0.0, 0.0, false);
            if (timerVal > delay + driveTime) {
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                System.out.println("Shooting"); // TODO: actually shoot
                timer.stop();
            }
        }
    }

}
