package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class AutonOption1 extends CommandBase {
    private final Drivetrain drive;
    private final Timer timer;
    private final double delay;
    private static boolean isAutoShootScheduled;
    private final AutoShoot autoShoot;
    private static final double DRIVE_TIME = 1.5;

    public AutonOption1(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        timer = new Timer();
        this.drive = drive;
        delay = SmartDashboard.getNumber("Shooting delay", 0.0);
        addRequirements(drive, shooter);
        autoShoot = new AutoShoot(drive, shooter, ingestor, null, null);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        isAutoShootScheduled = false;
        timer.reset();
        timer.start();
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
                if (!isAutoShootScheduled) {
                    isAutoShootScheduled = true;
                    CommandScheduler.getInstance().schedule(autoShoot);
                    //System.out.println("Shooting"); // TODO: actually shoot
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return autoShoot.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

}