package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class ManualShoot extends ShootCommand {
    public ManualShoot(Shooter shooter, Ingestor ingestor) {
        super(shooter, ingestor, 2300.0, 31.0);
        SmartDashboard.putNumber("Target RPM", targetRpm); // 2300 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        super.execute();
        //shooter.setShooterRpm(targetRpm);
        //shooter.setHoodAngle(targetAngle); // knob 2.5
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
