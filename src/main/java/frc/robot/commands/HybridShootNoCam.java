package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class HybridShootNoCam extends CommandBase {
    private final Shooter shooter;
    private final Ingestor ingestor;

    public HybridShootNoCam(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setHoodAngle(shooter.getTargetHoodAngle());
        shooter.setShooterRpm(shooter.getTargetRpm());
    }
    
}
