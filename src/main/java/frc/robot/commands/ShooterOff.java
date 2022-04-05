package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter;

public class ShooterOff extends CommandBase {
    private final Shooter shooter;

    public ShooterOff(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double rpm = 0.0;
        if (Shooter.getCoast()) {
            rpm = 2300.0;
        }
        shooter.setShooterRpm(rpm);
    }
}