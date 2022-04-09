package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class AutonSearch extends CommandBase{
    private final Drivetrain drive;
    private final Shooter shooter;
    private final Ingestor ingestor;
    private final AutoShoot autoShoot;

    public AutonSearch(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        this.drive = drive;
        this.shooter = shooter;
        this.ingestor = ingestor;
        autoShoot = new AutoShoot(drive, shooter, ingestor, null, null);
        addRequirements(drive, shooter, ingestor);
    }

    @Override
    public void initialize() {
        drive.resetCurrentStep();
    }

    @Override
    public void execute() {
        // TODO: when you add stuff remember to update isFinished()
        // spin right until we see a ball
        // center on ball
        // pick up ball
        // turn until we see a target
        // auto shoot
        // repeat or end?
    }

    @Override
    public boolean isFinished() {
        return drive.getCurrentStep() == 0; // change this if any setPosition steps are added in execute()
    }

    @Override
    public void end(boolean interrupted) {
        drive.swerveDrive(0, 0, 0, false);
        Shooter.setCoast(true);
        ingestor.liftIngestor();
        ingestor.stopIngestorWheels();
        ingestor.stopStage1();
        ingestor.stopStage2();
    }
    
}
