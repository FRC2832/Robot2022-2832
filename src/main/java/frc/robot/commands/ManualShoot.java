package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class ManualShoot extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;
    private double speed;

    public ManualShoot(Shooter shooter, Ingestor ingestor, double speed) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        this.speed = speed;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", speed); // 2300 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        // set shooter rpm
        double rpm = SmartDashboard.getNumber("Target RPM", speed);
        shooter.setShooterRpm(rpm);

        // set hood angle (knob 2.5, 66 degrees)
        // shooter.setHoodAngle(position);

        // if target rpm is within range (+- 50)
        if (rpm - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < rpm + 50) {
            ingestor.sendCargoToShooter();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setDefaultCommand(new NoShoot(shooter));
    }
}
