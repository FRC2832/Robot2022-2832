package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter;

public class HomeHood extends CommandBase {
    private Shooter shooter;

    public HomeHood(Shooter shooter) {
        this.shooter = shooter;
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
