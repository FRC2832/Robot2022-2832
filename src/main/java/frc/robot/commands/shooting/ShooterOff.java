package frc.robot.commands.shooting;


import frc.robot.subsystems.Shooter;

public class ShooterOff extends ShootCommand {

    public ShooterOff(Shooter shooter) {
        super(shooter);
    }

    @Override
    public void execute() {
        targetRpm = 0.0;
        if (Shooter.getCoast()) {
            targetRpm = 2300.0;
        }
        shooter.setShooterRpm(targetRpm);
    }
}