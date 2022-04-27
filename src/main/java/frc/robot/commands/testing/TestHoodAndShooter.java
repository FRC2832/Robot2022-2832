package frc.robot.commands.testing;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.shooting.DribbleShoot;
import frc.robot.commands.shooting.HubShoot;
import frc.robot.commands.shooting.SafeZoneShoot;
import frc.robot.commands.shooting.SideShoot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ControllerIO;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class TestHoodAndShooter extends CommandBase {
    private static final double ACCEPTABLE_RPM_MARGIN = 50.0;
    private static final double ACCEPTABLE_HOOD_MARGIN = 3.0;
    private final Ingestor ingestor;
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private final Climber climber;
    private final ColorSensor colorSensor;
    private final Timer commandTimer;
    private byte commandStep;
    private double currentTargetRpm;
    private double currentTargetAngle;
    private boolean isHolding;
    private boolean encounteredFailure;

    public TestHoodAndShooter(Ingestor ingestor, Shooter shooter, Drivetrain drivetrain, Climber climber, ColorSensor colorSensor) {
        super();
        this.ingestor = ingestor;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.climber = climber;
        this.colorSensor = colorSensor;
        addRequirements(ingestor, shooter, drivetrain, climber, colorSensor);
        commandTimer = new Timer();
    }

    @Override
    public void initialize() {
        commandStep = 0;
        isHolding = false;
        encounteredFailure = false;
        commandTimer.reset();
        shooter.setHoodCoastMode(NeutralMode.Brake);
        currentTargetRpm = 1000.0;
        currentTargetAngle = Shooter.MAX_ANGLE;

        SmartDashboard.putString("Current Test Routine", "TestHoodAndShooter Up");
    }

    @Override
    public void execute() {
        ingestor.stopAll();
        drivetrain.swerveDrive(0.0, 0.0, 0.0, false);
        // shooter.setShooterRpm(0.0);
        climber.arm1Hold();
        String currentTestStep;
        String nextTestStep;
        double nextTargetRpm;
        double nextTargetAngle;

        switch (commandStep) {
            case 0: // Move hood up to max angle (TestHoodAndShooter Up)
                currentTestStep = "Up";
                nextTestStep = "Down";
                nextTargetRpm = -1000.0;
                nextTargetAngle = Shooter.MIN_ANGLE;
                break;
            case 1: // Move hood down to min angle (TestHoodAndShooter Down)
                currentTestStep = "Down";
                nextTestStep = "Safe Zone";
                nextTargetRpm = SafeZoneShoot.SAFE_ZONE_TGT_RPM;
                nextTargetAngle = SafeZoneShoot.SAFE_ZONE_TGT_ANGLE;
                break;
            case 2: // Move hood to safe zone shot angle (TestHoodAndShooter Safe Zone)
                currentTestStep = "Safe Zone";
                nextTestStep = "Upper Hub";
                nextTargetRpm = HubShoot.UPPER_HUB_TGT_RPM;
                nextTargetAngle = HubShoot.UPPER_HUB_TGT_ANGLE;
                break;
            case 3: // Move hood to upper hub shot angle (TestHoodAndShooter Upper Hub)
                currentTestStep = "Upper Hub";
                nextTestStep = "Lower Hub";
                nextTargetRpm = HubShoot.LOWER_HUB_TGT_RPM;
                nextTargetAngle = HubShoot.LOWER_HUB_TGT_ANGLE;
                break;
            case 4: // Move hood to lower hub shot angle (TestHoodAndShooter Lower Hub)
                currentTestStep = "Lower Hub";
                nextTestStep = "Side Shot";
                nextTargetRpm = SideShoot.SIDE_SHOOT_TGT_RPM;
                nextTargetAngle = SideShoot.SIDE_SHOOT_TGT_ANGLE;
                break;
            case 5: // Move hood to side shot angle (TestHoodAndShooter Side Shot)
                currentTestStep = "Side Shot";
                nextTestStep = "Dribble Shot";
                nextTargetRpm = DribbleShoot.DRIBBLE_TARGET_RPM;
                nextTargetAngle = DribbleShoot.DRIBBLE_TARGET_ANGLE;
                break;
            case 6: // Move hood to dribble shot angle (TestHoodAndShooter Dribble Shot)
                currentTestStep = "Dribble Shot";
                nextTestStep = "(FINISHED)";
                nextTargetRpm = 0.0;
                nextTargetAngle = Shooter.MIN_ANGLE;
                break;
            default: // Catch all for anything else (should not reach here.)
                shooter.setShooterRpm(0.0);
                shooter.setHoodSpeedPct(0.0);
                return;
        }
        hoodAndShooterTest(nextTargetRpm, nextTargetAngle, nextTestStep, currentTestStep);
    }

    private void hoodAndShooterTest(double nextTargetRpm, double nextTargetAngle, String nextTestRoutine,
            String currTestRoutine) {
        if (isHolding) {
            shooter.setHoodSpeedPct(0.0);
            shooter.setShooterRpm(currentTargetRpm);
            if (commandTimer.get() >= 3.0) { // Three seconds elapsed. Move to the next step. Or end command if
                                             // encountered a failure.
                isHolding = false;
                if (encounteredFailure) {
                    commandStep = 7;
                    SmartDashboard.putString("Current Test Routine",
                            "TestHoodAndShooter " + currTestRoutine + " (FAILED)");
                } else {
                    commandStep++;
                    currentTargetRpm = nextTargetRpm;
                    currentTargetAngle = nextTargetAngle;
                    commandTimer.reset();
                    SmartDashboard.putString("Current Test Routine", "TestHoodAndShooter " + nextTestRoutine);
                }
            }
        } else {
            double currentShooterRpm = shooter.getShooterVelocity();
            double currentHoodAngle = shooter.getHoodAngle();
            if (Math.abs(currentTargetAngle - currentHoodAngle) <= ACCEPTABLE_HOOD_MARGIN
                && Math.abs(currentTargetRpm - currentShooterRpm) <= ACCEPTABLE_RPM_MARGIN) {
                // Hold the angle and RPM for three seconds.
                shooter.setShooterRpm(currentTargetRpm);
                shooter.setHoodSpeedPct(0.0);
                isHolding = true;
                commandTimer.reset();
            } else if (commandTimer.get() >= 10.0) {
                currentTargetRpm = 0.0;
                shooter.setShooterRpm(0.0);
                shooter.setHoodSpeedPct(0.0);
                encounteredFailure = true; // It shouldn't take this long. Something's probably wrong. Stop the test.
                isHolding = true;
                commandTimer.reset();
                ControllerIO instance = ControllerIO.getInstance();
                instance.rumbleDriveController(0.5);
                instance.rumbleOpController(0.5);
            } else {
                shooter.setShooterRpm(currentTargetRpm);
                shooter.setHoodAngle(currentTargetAngle);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        commandTimer.stop();
        drivetrain.swerveDrive(0.0, 0.0, 0.0, false);
        ingestor.stopAll();
        climber.arm1Hold();
        ControllerIO.getInstance().stopAllRumble();
        shooter.setHoodCoastMode(NeutralMode.Coast);
        shooter.setHoodSpeedPct(0.0);
        shooter.setShooterRpm(0.0);
    }

    @Override
    public boolean isFinished() {
        if (!ControllerIO.getInstance().isDrivePadConnected())
            return true;
        return commandStep > 6;
    }
}
