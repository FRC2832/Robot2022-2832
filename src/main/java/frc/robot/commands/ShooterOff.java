package frc.robot.commands;

import frc.robot.Shooter;

public class ShooterOff extends ShootCommand {

    public ShooterOff(Shooter shooter) {
        super(shooter, null, 0.0, shooter.getHoodAngle());
    }

    @Override
    public void execute() {
        targetRpm = 0.0;
        if (Shooter.getCoast()) {
            targetRpm = 2300.0;
        }
        super.execute();
    }
}