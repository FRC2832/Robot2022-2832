package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class DribbleShoot extends ShootCommand {
    private final Ingestor ingestor;

    public DribbleShoot(Shooter shooter, Ingestor ingestor) {
        super(shooter, 1000.0, 69.0);
        this.ingestor = ingestor;
        SmartDashboard.putNumber("Target RPM", 1000.0); // 1000 = dribble rpm
    }

    @Override
    public void execute() {
        // String detectedColor = colorSensor.getColor();
        // String allianceColor = pi.getAllianceColor().getString("default");
        super.execute();

        // set hood angle (knob 6, 39? degrees)
        //shooter.setHoodAngle(69.0);
        double shooterVel = shooter.getShooterVelocity();
        // if target rpm is within range (+- 50)
        if (targetRpm - 50 < shooterVel && shooterVel < targetRpm + 50) {
            ingestor.sendOneCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }

    @Override
    public boolean isFinished() {
        // when color is unknown return true (maybe wait one more second?)
        return ColorSensor.getCargoColor() == ColorSensor.CargoColor.Unknown;
    }
}
