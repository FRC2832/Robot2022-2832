package frc.robot.commands.shooting;

import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class HybridShootNoCam extends ShootCommand {
    private final Ingestor ingestor;

    public HybridShootNoCam(Shooter shooter, Ingestor ingestor) {
        super(shooter);
        this.ingestor = ingestor;
    }

    @Override
    public void execute() {
        targetHoodAngle = shooter.getTargetHoodAngle();
        targetRpm = shooter.getTargetRpm();
        super.execute();
    }
}
