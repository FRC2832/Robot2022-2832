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
import frc.robot.subsystems.ControllerIO;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class TestHood extends CommandBase {
    private static final double ACCEPTABLE_MARGIN = 3.0;
    private final Ingestor ingestor;
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private final Climber climber;
    private final Timer commandTimer;
    private byte commandStep;
    private double currentTargetAngle;
    private boolean isHolding;
    private boolean encounteredFailure;

    public TestHood(Ingestor ingestor, Shooter shooter, Drivetrain drivetrain, Climber climber) {
        super();
        this.ingestor = ingestor;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.climber = climber;
        addRequirements(ingestor, shooter, drivetrain, climber);
        commandTimer = new Timer();
    }

    @Override
    public void initialize() {
        commandStep = 0;
        isHolding = false;
        encounteredFailure = false;
        commandTimer.reset();
        shooter.setHoodCoastMode(NeutralMode.Brake);
        currentTargetAngle = Shooter.MAX_ANGLE;
        SmartDashboard.putString("Current Test Routine", "TestHood Up");
    }

    @Override
    public void execute() {
        ingestor.stopAll();
        drivetrain.swerveDrive(0.0, 0.0, 0.0, false);
        shooter.setShooterRpm(0.0);
        climber.arm1Hold();
        String currHoodTestStep;
        String nextHoodTestStep;
        double nextTargetAngle;

        switch (commandStep) {
            case 0: // Move hood up to max angle (TestHood Up)
                currHoodTestStep = "Up";
                nextHoodTestStep = "Down";
                nextTargetAngle = Shooter.MIN_ANGLE;
                break;
            case 1: // Move hood down to min angle (TestHood Down)
                currHoodTestStep = "Down";
                nextHoodTestStep = "Safe Zone";
                nextTargetAngle = SafeZoneShoot.SAFE_ZONE_TGT_ANGLE;
                break;
            case 2: // Move hood to safe zone shot angle (TestHood Safe Zone)
                currHoodTestStep = "Safe Zone";
                nextHoodTestStep = "Upper Hub";
                nextTargetAngle = HubShoot.UPPER_HUB_TGT_ANGLE;
                break;
            case 3: // Move hood to upper hub shot angle (TestHood Upper Hub)
                currHoodTestStep = "Upper Hub";
                nextHoodTestStep = "Lower Hub";
                nextTargetAngle = HubShoot.LOWER_HUB_TGT_ANGLE;
                break;
            case 4: // Move hood to lower hub shot angle (TestHood Lower Hub)
                currHoodTestStep = "Lower Hub";
                nextHoodTestStep = "Side Shot";
                nextTargetAngle = SideShoot.SIDE_SHOOT_TGT_ANGLE;
                break;
            case 5: // Move hood to side shot angle (TestHood Side Shot)
                currHoodTestStep = "Side Shot";
                nextHoodTestStep = "Dribble Shot";
                nextTargetAngle = DribbleShoot.DRIBBLE_TARGET_ANGLE;
                break;
            case 6: // Move hood to dribble shot angle (TestHood Dribble Shot)
                currHoodTestStep = "Dribble Shot";
                nextHoodTestStep = "(FINISHED)";
                nextTargetAngle = Shooter.MIN_ANGLE;
                break;
            default: // Catch all for anything else (should not reach here.)
                shooter.setHoodSpeedPct(0.0);
                return;
        }
        hoodTest(nextTargetAngle, nextHoodTestStep, currHoodTestStep);
    }

    private void hoodTest(double nextTargetAngle, String nextTestRoutine, String currTestRoutine) {
        if (isHolding) {
            shooter.setHoodSpeedPct(0.0);
            if (commandTimer.get() >= 3.0) { // Three seconds elapsed. Move to the next step. Or end command if
                                             // encountered a failure.
                isHolding = false;
                if (encounteredFailure) {
                    commandStep = 7;
                    SmartDashboard.putString("Current Test Routine", "TestHood " + currTestRoutine + " (FAILED)");
                } else {
                    commandStep++;
                    currentTargetAngle = nextTargetAngle;
                    commandTimer.reset();
                    SmartDashboard.putString("Current Test Routine", "TestHood " + nextTestRoutine);
                }
            }
        } else {
            double currentHoodAngle = shooter.getHoodAngle();
            if (Math.abs(currentTargetAngle - currentHoodAngle) <= ACCEPTABLE_MARGIN) {
                // Hold the angle for three seconds.
                shooter.setHoodSpeedPct(0.0);
                isHolding = true;
                commandTimer.reset();
            } else if (commandTimer.get() >= 10.0) {
                shooter.setHoodSpeedPct(0.0);
                encounteredFailure = true; // It shouldn't take this long to move the hood. Something's probably wrong.
                isHolding = true;
                commandTimer.reset();
                ControllerIO instance = ControllerIO.getInstance();
                instance.rumbleDriveController(0.5);
                instance.rumbleOpController(0.5);
            } else {
                shooter.setHoodAngle(currentTargetAngle);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        commandTimer.stop();
        ControllerIO.getInstance().stopAllRumble();
        drivetrain.swerveDrive(0.0, 0.0, 0.0, false);
        ingestor.stopAll();
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
