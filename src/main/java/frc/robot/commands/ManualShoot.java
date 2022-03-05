package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter;

public class ManualShoot extends CommandBase {
    private Shooter shooter;

    public ManualShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", 2300); // 2300 = sweet spot based on '2022 shooter speed table'
    }
    
    @Override
    public void execute() {
        // set shooter rpm
        double rpm = SmartDashboard.getNumber("Target RPM", 2300);
        shooter.setShooterRpm(rpm);

        // set hood angle
        // shooter.setHoodAngle(position);

        // if target rpm is within range (+- 30)
        if(rpm - 30 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < rpm + 30){
            // start treads in slipstream
        }
    }
}
