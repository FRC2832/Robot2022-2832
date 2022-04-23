package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetHood extends CommandBase {
    private final Shooter shooter;
    // private double targetAngle;

    public SetHood(Shooter shooter, double targetAngle) {
        super();
        this.shooter = shooter;
        // this.targetAngle = targetAngle;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setHoodSpeedPct(-0.25);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodSpeedPct(0);
    }

    @Override
    public boolean isFinished() {
        return shooter.isHoodHomed();
    }
}
