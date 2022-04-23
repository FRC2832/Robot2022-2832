package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class SideShoot extends ShootCommand {
    private final Ingestor ingestor;

    public SideShoot(Shooter shooter, Ingestor ingestor) {
        super(shooter, 2300.0, 31.0);
        this.ingestor = ingestor;
        SmartDashboard.putNumber("Target RPM", targetRpm); // 2300 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        //SPEED = SmartDashboard.getNumber("Target RPM", 2300.0); // Do not uncomment unless calibrating hood.
        super.execute(); // Target hood angle: knob 2.5
        double shooterVel = shooter.getShooterVelocity();
        // if target rpm is within range (+- 50)
        if (targetRpm - 50 < shooterVel && shooterVel < targetRpm + 50) {
            double hoodAngle = shooter.getHoodAngle();
            if (targetHoodAngle - 3.0 < hoodAngle && hoodAngle < targetHoodAngle + 3.0) {
                ingestor.sendCargoToShooter();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
