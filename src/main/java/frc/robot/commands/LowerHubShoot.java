package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class LowerHubShoot extends CommandBase {
    private static final double SPEED = 1000.0;
    private final Shooter shooter;
    private final Ingestor ingestor;

    public LowerHubShoot(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", SPEED);
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(SPEED);
        shooter.setHoodAngle(69.0); // knob 6
        double shooterVel = shooter.getShooterVelocity();
        // if target rpm is within range (+- 50)
        if (SPEED - 50 < shooterVel && shooterVel < SPEED + 50) {
            ingestor.sendCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
