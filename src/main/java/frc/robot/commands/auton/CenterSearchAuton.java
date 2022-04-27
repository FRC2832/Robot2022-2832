package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.*;
import frc.robot.commands.driving.CenterToCargo;
import frc.robot.commands.shooting.AutoShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Pi;
import frc.robot.subsystems.Shooter;

public class CenterSearchAuton extends CommandBase {
    private final Drivetrain drive;
    private final Ingestor ingestor;
    private final AutoShoot autoShoot;
    private final CenterToCargo centerToCargo;
    private final Timer timer;
    private final SwerveModule frontLeft;
    private boolean isCargoScheduled;
    private boolean isAutoShootScheduled;
    private boolean firstTurn;
    private double startAngle;
    private double startEncoderCount;
    private int direction;

    public CenterSearchAuton(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        super();
        this.drive = drive;
        this.ingestor = ingestor;
        autoShoot = new AutoShoot(drive, shooter, ingestor, false);
        centerToCargo = new CenterToCargo(drive);
        timer = new Timer();
        frontLeft = drive.getModules()[0];
        addRequirements(drive, ingestor);
    }

    @Override
    public void initialize() {
        drive.resetCurrentStep();
        isCargoScheduled = false;
        isAutoShootScheduled = false;
        firstTurn = true;
        startAngle = drive.getAngle() % 360;
        startEncoderCount = frontLeft.getDistance();
        direction = 1;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double distance;
        // System.out.println("Drive Step: " + drive.getCurrentStep());
        switch (drive.getCurrentStep()) {
            case 0: // back up off the tarmac
                drive.swerveDrive(Drivetrain.kMaxSpeed / 4, 0.0, 0.0, false);
                ingestor.liftIngestor();
                distance = Math.abs(frontLeft.getDistance() - startEncoderCount);
                if (distance >= 1.2) {
                    drive.incrementCurrentStep();
                }
                break;
            case 1: // auto shoot
                if (!isAutoShootScheduled) {
                    CommandScheduler.getInstance().schedule(autoShoot);
                    isAutoShootScheduled = true;
                }
                if (Robot.getIsAutoShootFinished()) {
                    // System.out.println("is auto shoot finished = true, incrementing drive step");
                    drive.incrementCurrentStep();
                    isAutoShootScheduled = false;
                    Robot.setIsAutoShootFinished(false);
                    timer.reset();
                    startAngle = drive.getAngle() % 360;
                }
                break;
            case 2: // spin right then left until we see a ball
                // if (drive.getAngle() > startAngle + 20) {
                //     direction = -1;
                // } else if (drive.getAngle() < startAngle - 20) {
                //     direction = 1;
                // }
                if (timer.get() > 2 && firstTurn) {
                    direction *= -1;
                    timer.reset();
                    firstTurn = false;
                } else if (timer.get() > 4 && !firstTurn) {
                    direction *= -1;
                    timer.reset();
                }
                // System.out.println("Start angle: " + startAngle + "\tCurrent angle: " + drive.getAngle());
                drive.swerveDrive(0.0, 0.0, direction * 0.6981317007977318, false);
                ingestor.liftIngestor();
                if (Pi.getCargoCenterY() > 0.0) { // TODO: check max y distance we can look at without going too far
                    drive.incrementCurrentStep();
                }
                break;
            case 3: // center on ball
                if (!isCargoScheduled) {
                    CommandScheduler.getInstance().schedule(centerToCargo);
                    isCargoScheduled = true;
                }
                ingestor.liftIngestor();
                if (centerToCargo.isFinished()) {
                    timer.reset();
                    drive.incrementCurrentStep();
                }
                break;
            case 4: // drive to and ingest ball
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                drive.swerveDrive(-Drivetrain.kMaxSpeed / 4, 0.0, 0.0, false);
                // TODO: can we relate timer failsafe time to y distance of cargo?
                if (ingestor.getStage1Proximity() || timer.get() > 1) { // TODO: check max time
                    drive.swerveDrive(0.0, 0.0, 0.0, false);
                    timer.reset();
                    drive.incrementCurrentStep();
                }
                break;
            case 5: // turn until we see a target
                // TODO: consider backing up first for however long we drove to get the first ball
                ingestor.liftIngestor();
                ingestor.runStage1In(); // in case the ball didn't get all the way in
                drive.swerveDrive(0.0, 0.0, -0.6981317007977318, false);
                if (Pi.getTargetCenterX() > 0.0) {
                    drive.swerveDrive(0.0, 0.0, 0.0, false);
                    drive.incrementCurrentStep();
                }
                break;
            case 6: // auto shoot
                if (!isAutoShootScheduled) {
                    CommandScheduler.getInstance().schedule(autoShoot);
                    isAutoShootScheduled = true;
                }
                if (autoShoot.isFinished()) {
                    drive.incrementCurrentStep();
                }
                break;
            default:
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                ingestor.liftIngestor();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.swerveDrive(0.0, 0.0, 0.0, false);
        Shooter.setCoast(true);
        ingestor.stopAll();
        isCargoScheduled = false;
        isAutoShootScheduled = false;
    }

    @Override
    public boolean isFinished() {
        return drive.getCurrentStep() == 7; // change this if any setPosition steps are added in execute()
    }

}
