package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class SideShoot extends CommandBase {
    private static final double SPEED = 2300.0;
    private static final double ANGLE = 31.0;
    private final Shooter shooter;
    private final Ingestor ingestor;

    public SideShoot(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", SPEED); // 2300 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        //SPEED = SmartDashboard.getNumber("Target RPM", 2300.0); // Do not uncomment unless calibrating hood.
        shooter.setShooterRpm(SPEED);
        shooter.setHoodAngle(ANGLE); // knob 2.5
        double shooterVel = shooter.getShooterVelocity();
        // if target rpm is within range (+- 50)
        if (SPEED - 50 < shooterVel && shooterVel < SPEED + 50) {
            double hoodAngle = shooter.getHoodAngle();
            if (ANGLE - 3.0 < hoodAngle && hoodAngle < ANGLE + 3.0) {
                ingestor.sendCargoToShooter();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
