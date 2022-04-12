package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Pi;
import frc.robot.Shooter;
import frc.robot.SwerveModule;

public class AutonCenterSearch extends CommandBase{
    private final Drivetrain drive;
    private final Ingestor ingestor;
    private final AutoShoot autoShoot;
    private final CenterToCargo centerToCargo;
    private final Timer timer;
    private final SwerveModule frontLeft;
    private boolean isCargoScheduled;
    private boolean isAutoShootScheduled;
    private boolean autoShootFinished; // (move this variable to robot.java) set this variable from autoshoot instead of calling autoShoot.isFinished() (im wondering if thats calling a different object than what is schedule for some weird reason)
    private double startAngle;
    private double startEncoderCount;
    private int direction;

    public AutonCenterSearch(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        this.drive = drive;
        this.ingestor = ingestor;
        autoShoot = new AutoShoot(drive, shooter, ingestor, null, null);
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
        startAngle = drive.getAngle() % 360;
        startEncoderCount = frontLeft.getDistance();
        direction = 1;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double distance;
        System.out.println("Drive Step: " + drive.getCurrentStep());
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
                if (autoShoot.isFinished()) {
                    drive.incrementCurrentStep();
                    isAutoShootScheduled = false;
                }
                break;
            case 2: // spin right then left until we see a ball
                if (drive.getAngle() > startAngle + 20) {
                    direction = -1;
                } else if (drive.getAngle() < startAngle - 20) {
                    direction = 1;
                }
                drive.swerveDrive(0.0, 0.0, direction * Math.toRadians(40), false);
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
                if(ingestor.getStage1Proximity() || timer.get() > 2.5) { // TODO: check max time
                    drive.swerveDrive(0.0, 0.0, 0.0, false);
                    timer.reset();
                    drive.incrementCurrentStep();
                }
                break;
            case 5: // turn until we see a target
                // TODO: consider backing up first for however long we drove to get the first ball
                ingestor.liftIngestor();
                ingestor.runStage1In(); // in case the ball didn't get all the way in
                drive.swerveDrive(0.0, 0.0, -Math.toRadians(40), false);
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
    public boolean isFinished() {
        return drive.getCurrentStep() == 7; // change this if any setPosition steps are added in execute()
    }

    @Override
    public void end(boolean interrupted) {
        drive.swerveDrive(0, 0, 0, false);
        Shooter.setCoast(true);
        ingestor.liftIngestor();
        ingestor.stopIngestorWheels();
        ingestor.stopStage1();
        ingestor.stopStage2();
        isCargoScheduled = false;
        isAutoShootScheduled = false;
    }
    
}
