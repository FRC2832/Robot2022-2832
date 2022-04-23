package frc.robot.commands.shooting;

import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class ShooterBackwards extends ShootCommand {
    private final Ingestor ingestor;

    public ShooterBackwards(Shooter shooter, Ingestor ingestor) {
        super(shooter, -1000.0);
        this.ingestor = ingestor;
    }

    @Override
    public void execute() {
        ingestor.lowerIngestor();
        ingestor.runIngestorOut();
        shooter.setShooterRpm(targetRpm);
        ingestor.runStage2Out();
        ingestor.runStage1Out();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterRpm(0.0);
        ingestor.stopStage2();
        ingestor.stopStage1();
    }

}