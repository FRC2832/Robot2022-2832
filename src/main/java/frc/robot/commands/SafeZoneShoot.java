package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class SafeZoneShoot extends CommandBase {
    private static final double SPEED = 2650.0;
    private static final double ANGLE = 53.0;
    private final Shooter shooter;
    private final Ingestor ingestor;
    private final boolean changeHood;

    public SafeZoneShoot(Shooter shooter, Ingestor ingestor, boolean changeHood) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        this.changeHood = changeHood;
        //speed = 2650.0;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", SPEED); // 2650 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(SPEED);

        if (changeHood) {
            shooter.setHoodAngle(53.0); // knob 4.5
        }
        double shooterVel = shooter.getShooterVelocity();
        if (SPEED - 30 < shooterVel && shooterVel < SPEED + 30) {
            if (changeHood) { // If we don't need to change the hood, skip the angle check.
                double hoodAngle = shooter.getHoodAngle();
                if (ANGLE - 3 >= hoodAngle || hoodAngle >= ANGLE + 3) {
                    return; // Hood is not angled correctly yet, don't shoot.
                }
            }
            ingestor.sendCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
