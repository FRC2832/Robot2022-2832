package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class ShooterBackwards extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;

    public ShooterBackwards(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(-1000);
        ingestor.runStage2Out();
        ingestor.runStage1Out();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterRpm(0);
        ingestor.stopStage2();
        ingestor.stopStage1();
    }

}