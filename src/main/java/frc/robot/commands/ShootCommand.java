package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public abstract class ShootCommand extends CommandBase {
    protected Shooter shooter;
    protected Ingestor ingestor;
    protected double targetRpm;
    protected double targetAngle;

    protected ShootCommand(Shooter shooter, Ingestor ingestor, double targetRpm, double targetAngle) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        this.targetRpm = targetRpm;
        this.targetAngle = targetAngle;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(targetRpm);
        shooter.setHoodAngle(targetAngle);
    }
}