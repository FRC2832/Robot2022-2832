package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class SafeZoneShoot extends ShootCommand {
    public static final double SAFE_ZONE_TGT_ANGLE = 53.0;
    private final Ingestor ingestor;
    private final boolean changeHood;

    public SafeZoneShoot(Shooter shooter, Ingestor ingestor, boolean changeHood) {
        super(shooter, 2650.0, SAFE_ZONE_TGT_ANGLE);
        this.ingestor = ingestor;
        this.changeHood = changeHood;
        SmartDashboard.putNumber("Target RPM", targetRpm); // 2650 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(targetRpm);

        if (changeHood) {
            shooter.setHoodAngle(targetHoodAngle); // knob 4.5
        }
        double shooterVel = shooter.getShooterVelocity();
        if (targetRpm - 30 < shooterVel && shooterVel < targetRpm + 30) {
            if (changeHood) { // If we don't need to change the hood, skip the angle check.
                double hoodAngle = shooter.getHoodAngle();
                if (targetHoodAngle - 3 >= hoodAngle || hoodAngle >= targetHoodAngle + 3) {
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
