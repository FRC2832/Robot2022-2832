package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class SafeZoneShoot extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;
    private double speed;
    private boolean changeHood;

    public SafeZoneShoot(Shooter shooter, Ingestor ingestor, boolean changeHood) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        this.changeHood = changeHood;
        speed = 2650;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", speed); // 2650 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(speed);

        if (changeHood) {
            shooter.setHoodAngle(53); // knob 4.5
        }
        double shooterVel = shooter.getShooterVelocity();
        if (speed - 30 < shooterVel && shooterVel < speed + 30) {
            ingestor.sendCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
