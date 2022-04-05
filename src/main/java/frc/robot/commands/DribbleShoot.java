package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ColorSensor;
import frc.robot.ColorSensor.CargoColor;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class DribbleShoot extends ShootCommand {
    // private Pi pi;

    public DribbleShoot(Shooter shooter, Ingestor ingestor) {
        super(shooter, ingestor, 1000, 69.0);
        // this.pi = pi;
        SmartDashboard.putNumber("Target RPM", 1000); // 1000 = dribble rpm
    }

    @Override
    public void execute() {
        // String detectedColor = colorSensor.getColor();
        // String allianceColor = pi.getAllianceColor().getString("default");
        super.execute();
        //double rpm = 1000.0;
        //shooter.setShooterRpm(targetRpm);

        // set hood angle (knob 6, 39? degrees)
        //shooter.setHoodAngle(targetAngle);
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
        return ColorSensor.getCargoColor() == CargoColor.Unknown;
    }
}
