package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class HubShoot extends ShootCommand {
    private final Ingestor ingestor;
    private final boolean isUpper;

    public HubShoot(Shooter shooter, Ingestor ingestor, boolean isUpper) {
        super(shooter);
        this.ingestor = ingestor;
        this.isUpper = isUpper;
    }

    @Override
    public void initialize() {
        if (isUpper) {
            targetRpm = 2150.0; // Upper hub
            targetHoodAngle = 18.0;
        } else {
            targetRpm = 1000.0; // Lower hub
            targetHoodAngle = 69.0;
        }
        SmartDashboard.putNumber("Target RPM", targetRpm);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }

    @Override
    public void execute() {
        super.execute(); // Target hood angle: knob 6
        double shooterVel = shooter.getShooterVelocity();
        // if target rpm is within range (+- 50)
        if (targetRpm - 50 < shooterVel && shooterVel < targetRpm + 50) {
            if (isUpper) {
                double shooterAngle = shooter.getHoodAngle();
                if (targetHoodAngle - 3.0 < shooterAngle && shooterAngle < targetHoodAngle + 3.0) {
                    ingestor.sendCargoToShooter();
                }
            } else {
                ingestor.sendCargoToShooter();
            }
        }
    }
}
