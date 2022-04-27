package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AwaitControllerInput extends CommandBase {
    private final Drivetrain drivetrain;
    private final Climber climber;
    private final Ingestor ingestor;
    private final Shooter shooter;
    private final Timer timer;
    private boolean isRumbling;

    public AwaitControllerInput(Drivetrain drivetrain, Climber climber, Ingestor ingestor, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.climber = climber;
        this.ingestor = ingestor;
        this.shooter = shooter;
        addRequirements(drivetrain, climber, ingestor, shooter);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        isRumbling = false;
    }

    @Override
    public void execute() {
        drivetrain.swerveDrive(0.0, 0.0, 0.0, false);
        climber.arm1Hold();
        ingestor.stopAll();
        shooter.setHoodSpeedPct(0.0);
        shooter.setShooterRpm(0.0);
        if (timer.get() >= 1.0) { // Rumble the controllers at one-second intervals.
            timer.reset();
            ControllerIO instance = ControllerIO.getInstance();
            if (isRumbling) {
                instance.stopAllRumble();
            } else {
                instance.rumbleDriveController(0.5);
                instance.rumbleOpController(0.5);
            }
            isRumbling = !isRumbling;
        }

    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        ControllerIO.getInstance().stopAllRumble();
        isRumbling = false;
    }

    @Override
    public boolean isFinished() {
        ControllerIO instance = ControllerIO.getInstance();
        return instance.isDriveABtnPressed() || instance.isOpAButtonPressed();
    }
}
