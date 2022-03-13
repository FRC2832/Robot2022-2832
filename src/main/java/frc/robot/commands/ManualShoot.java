package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class ManualShoot extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;
    private double speed;

    public ManualShoot(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        speed = 2300;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", speed); // 2300 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        shooter.setShooterRpm(speed);

        // set hood angle (knob 2.5, 66 degrees)
        // shooter.setHoodAngle(position);

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
