package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public abstract class ShootCommand extends CommandBase {
    protected Shooter shooter;
    protected double targetRpm;
    double targetHoodAngle;

    protected ShootCommand(Shooter shooter) {
        super();
        this.shooter = shooter;
        addRequirements(shooter);
    }

    protected ShootCommand(Shooter shooter, double targetRpm) {
        super();
        this.shooter = shooter;
        this.targetRpm = targetRpm;
        this.targetHoodAngle = shooter.getHoodAngle();
        addRequirements(shooter);
    }

    protected ShootCommand(Shooter shooter, double targetRpm, double targetHoodAngle) {
        super();
        this.shooter = shooter;
        this.targetHoodAngle = targetHoodAngle;
        this.targetRpm = targetRpm;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(targetRpm);
        shooter.setHoodAngle(targetHoodAngle);
    }
}
