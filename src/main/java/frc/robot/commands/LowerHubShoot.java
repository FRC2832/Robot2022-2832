package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class LowerHubShoot extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;
    private double speed;

    public LowerHubShoot(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        speed = 1000;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", speed);
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(speed);
        shooter.setHoodAngle(69); // knob 6

        // if target rpm is within range (+- 50)
        if (speed - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < speed + 50) {
            ingestor.sendCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
