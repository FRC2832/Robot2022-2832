package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter;

public class ShooterOff extends CommandBase {
    private Shooter shooter;

    public ShooterOff(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (Shooter.getCoast()) {
            shooter.setShooterRpm(1000);
        } else {
            shooter.setShooterRpm(0);
        }
    }
}