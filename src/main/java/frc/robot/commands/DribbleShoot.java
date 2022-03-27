package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ColorSensor;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class DribbleShoot extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;
    //private Pi pi;

    public DribbleShoot(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        //this.pi = pi;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", 1000); // 1000 = dribble rpm
    }

    @Override
    public void execute() {
        // String detectedColor = colorSensor.getColor();
        // String allianceColor = pi.getAllianceColor().getString("default");

        double rpm = 1000.0;
        shooter.setShooterRpm(rpm);

        // set hood angle (knob 6, 39? degrees)
        shooter.setHoodAngle(69);

        // if target rpm is within range (+- 50)
        if (rpm - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < rpm + 50) {
            ingestor.sendOneCargoToShooter();
        }
    }

    @Override
    public boolean isFinished() {
        // wehn color is unknown return true (maybe wait one more second?)
        if (ColorSensor.getCargoColor().equals("Unknown")) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
