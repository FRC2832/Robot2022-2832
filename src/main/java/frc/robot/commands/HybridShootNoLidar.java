package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class HybridShootNoLidar extends CommandBase {
    private final Shooter shooter;
    private final Ingestor ingestor;
    private final Drivetrain drive;

    public HybridShootNoLidar(Shooter shooter, Ingestor ingestor, Drivetrain drive) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        this.drive = drive;
        addRequirements(shooter, drive);
    }

    @Override
    public void execute() {
        shooter.setHoodAngle(shooter.getTargetHoodAngle());
        shooter.setShooterRpm(shooter.getTargetRpm());
    }
    
}
