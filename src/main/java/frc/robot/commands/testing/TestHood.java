package frc.robot.commands.testing;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class TestHood extends CommandBase {
    private final Ingestor ingestor;
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private final Climber climber;
    private final ColorSensor colorSensor;
    private final XboxController driverController;
    private final XboxController operatorController;
    private final Timer commandTimer;

    public TestHood(Ingestor ingestor, Shooter shooter, Drivetrain drivetrain, Climber climber, ColorSensor colorSensor, XboxController driverController, XboxController operatorController) {
        super();
        this.ingestor = ingestor;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.climber = climber;
        this.colorSensor = colorSensor;
        this.driverController = driverController;
        this.operatorController = operatorController;
        addRequirements(ingestor, shooter, drivetrain, climber, colorSensor);
        commandTimer = new Timer();
    }

    @Override
    public void initialize() {
        commandTimer.reset();
        shooter.setHoodCoastMode(NeutralMode.Brake);
    }

    @Override
    public void execute() {
        ingestor.stopAll();
        drivetrain.swerveDrive(0.0, 0.0, 0.0, false);
        shooter.setShooterRpm(0.0);
        climber.arm1Hold();

    }

    @Override
    public boolean isFinished() {
        if (!driverController.isConnected())
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        commandTimer.stop();
        Robot.stopControllerRumble(driverController);
        Robot.stopControllerRumble(operatorController);
        shooter.setHoodCoastMode(NeutralMode.Coast);
    }
}
