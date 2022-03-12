package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class SafeZoneShoot extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;
    private double speed;

    public SafeZoneShoot(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        speed = 2650;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", speed); // 2650 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(speed);

        // set hood angle (knob 4.5, 48 degrees)
        // shooter.setHoodAngle(position);

        if (speed - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < speed + 50) {
            ingestor.sendCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setDefaultCommand(new NoShoot(shooter));
    }
}
