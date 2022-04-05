package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class HubShoot extends ShootCommand {
    public HubShoot(Shooter shooter, Ingestor ingestor, boolean isUpper) {
        super(shooter, ingestor, 1000.0, 69.0); // Assume lower hub shot at first. Change later if needed.
        if (isUpper) {
            targetRpm = 2150.0; // Upper hub
            targetAngle = 18.0;
        }
        SmartDashboard.putNumber("Target RPM", targetRpm);
    }

    @Override
    public void execute() {
        super.execute();
        //shooter.setShooterRpm(targetRpm);
        //shooter.setHoodAngle(targetAngle); // knob 6
        double shooterVel = shooter.getShooterVelocity();
        // if target rpm is within range (+- 50)
        if (targetRpm - 50 < shooterVel && shooterVel < targetRpm + 50) {
            ingestor.sendCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
