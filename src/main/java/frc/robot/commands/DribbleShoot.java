package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class DribbleShoot extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;
    private Timer timer;
    private boolean cargoSentToShooter;
    // private Pi pi;

    public DribbleShoot(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        timer = new Timer();
        // this.pi = pi;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", 1000); // 1000 = dribble rpm
    }

    @Override
    public void initialize() {
        timer.reset();
        cargoSentToShooter = false;
    }

    @Override
    public void execute() {
        // String detectedColor = colorSensor.getColor();
        // String allianceColor = pi.getAllianceColor().getString("default");

        double rpm = 1000.0;
        shooter.setShooterRpm(rpm);

        // set hood angle (knob 6, 39? degrees)
        shooter.setHoodAngle(69.0);

        // if target rpm is within range (+- 50)
        if (rpm - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < rpm + 50) {
            if (ingestor.sendOneCargoToShooter() && !cargoSentToShooter) {
                cargoSentToShooter = true;
                timer.start();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // wehn color is unknown return true (maybe wait one more second?)
        //System.out.println("Current ALLIANCE color: " + DriverStation.getAlliance() + "\nCurrent  DRIBBLE SHOOT color sensor value: " + ColorSensor.getCargoColor()))
        //CargoColor cargoColor = ColorSensor.getCargoColor();
        return cargoSentToShooter && timer.get() >= 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
        timer.stop();
        timer.reset();
    }
}
