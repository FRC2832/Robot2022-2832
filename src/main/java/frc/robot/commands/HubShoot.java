package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class HubShoot extends CommandBase {
    private final double speed;
    private final double angle;
    private final Shooter shooter;
    private final Ingestor ingestor;
    private final boolean isUpper;

    public HubShoot(Shooter shooter, Ingestor ingestor, boolean isUpper) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        this.isUpper = isUpper;
        addRequirements(shooter);
        if (isUpper) {
            speed = 2150.0; // Upper hub
            angle = 18.0;
        } else {
            speed = 1000.0; // Lower hub
            angle = 69.0;
        }
        SmartDashboard.putNumber("Target RPM", speed);
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(speed);
        shooter.setHoodAngle(angle); // knob 6
        double shooterVel = shooter.getShooterVelocity();
        // if target rpm is within range (+- 50)
        if (speed - 50 < shooterVel && shooterVel < speed + 50) {
            if (isUpper) {
                double shooterAngle = shooter.getHoodAngle();
                if (angle - 3.0 < shooterAngle && shooterAngle < angle + 3.0) {
                    ingestor.sendCargoToShooter();
                }
            } else {
                ingestor.sendCargoToShooter();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
