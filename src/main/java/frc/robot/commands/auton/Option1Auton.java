package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.shooting.AutoShoot;

public class Option1Auton extends CommandBase {
    private static final double DRIVE_TIME = 1.5;
    private static boolean isAutoShootScheduled;
    private final Drivetrain drive;
    private final Timer timer;
    private final double delay;
    private final AutoShoot autoShoot;

    public Option1Auton(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        super();
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
            drive.swerveDrive(-1.0, 0.0, 0.0, false);
        } else {
            drive.swerveDrive(0.0, 0.0, 0.0, false);
            if (timerVal > delay + DRIVE_TIME) {
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                if (!isAutoShootScheduled) {
                    isAutoShootScheduled = true;
                    CommandScheduler.getInstance().schedule(autoShoot);
                    //System.out.println("Shooting"); // TODO: actually shoot
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return autoShoot.isFinished();
    }

}