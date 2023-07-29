package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter;

public class ShooterOff extends CommandBase {
    private final Shooter shooter;
    private final XboxController operator;

    public ShooterOff(Shooter shooter, XboxController operator) {
        this.shooter = shooter;
        this.operator = operator;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double rpm = 0.0;
        if(operator.getLeftTriggerAxis() > 0.5) {
            rpm = -4500;
        }
        else if (Shooter.getCoast()) {
            rpm = 2300.0;
        }
        shooter.setShooterRpm(rpm);
    }
}