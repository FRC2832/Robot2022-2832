package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter;

public class SetHood extends CommandBase {
    private Shooter shooter;
    private double targetAngle;

    public SetHood(Shooter shooter, double targetAngle) {
        this.shooter = shooter;
        this.targetAngle = targetAngle;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.setHoodSpeedPct(-0.25);
    }

    @Override
    public boolean isFinished() {
        return shooter.isHoodHomed();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodSpeedPct(0);
    }
}
