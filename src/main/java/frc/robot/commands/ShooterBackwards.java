package frc.robot.commands;

import frc.robot.Ingestor;
import frc.robot.Shooter;

public class ShooterBackwards extends ShootCommand {
    public ShooterBackwards(Shooter shooter, Ingestor ingestor) {
        super(shooter, ingestor, -1000.0, shooter.getHoodAngle());
    }

    @Override
    public void execute() {
        super.execute();
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