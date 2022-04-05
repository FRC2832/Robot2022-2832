package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class SafeZoneShoot extends ShootCommand {
    private static final double TARGET_SHOOTER_SPEED = 2650.0;

    public SafeZoneShoot(Shooter shooter, Ingestor ingestor, boolean changeHood) {
        super(shooter, ingestor, TARGET_SHOOTER_SPEED, shooter.getHoodAngle());
        if (changeHood) {
            targetAngle = 53.0;
        }
        // speed = 2650.0;
        SmartDashboard.putNumber("Target RPM", TARGET_SHOOTER_SPEED); // 2650 = sweet spot based on '2022 shooter speed
                                                                      // table'
    }

    @Override
    public void execute() {
        super.execute();
        double shooterVel = shooter.getShooterVelocity();
        if (TARGET_SHOOTER_SPEED - 30 < shooterVel && shooterVel < TARGET_SHOOTER_SPEED + 30) {
            ingestor.sendCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
