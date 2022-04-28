package frc.robot.commands.testing;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControllerIO;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Shooter;

public class TestAutoShoot extends CommandBase {
    private static final double ACCEPTABLE_RPM_MARGIN = 50.0;
    private static final double ACCEPTABLE_HOOD_MARGIN = 3.0;
    private final Ingestor ingestor;
    private final Shooter shooter;
    private final Drivetrain drivetrain;
    private final Climber climber;
    private final Timer commandTimer;
    private final ArrayList<Pair<Double, Double>> rpmList;
    private final ArrayList<Pair<Double, Double>> angleList;
    private int tableIndex;
    private double currentTargetRpm;
    private double currentTargetAngle;
    private boolean isHolding;
    private boolean encounteredFailure;

    public TestAutoShoot(Ingestor ingestor, Shooter shooter, Drivetrain drivetrain, Climber climber) {
        super();
        this.ingestor = ingestor;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.climber = climber;
        addRequirements(ingestor, shooter, drivetrain, climber);
        commandTimer = new Timer();
        rpmList = ShooterConstants.getDistRpmTable();
        angleList = ShooterConstants.getDistHoodTable();
    }

    @Override
    public void initialize() {
        tableIndex = 0;
        isHolding = false;
        encounteredFailure = false;
        commandTimer.reset();
        shooter.setHoodCoastMode(NeutralMode.Brake);
        currentTargetRpm = rpmList.get(0).getSecond();
        currentTargetAngle = angleList.get(0).getSecond();

        SmartDashboard.putString("Current Test Routine", "TestAutoShoot " + currentTargetRpm + " rpm, " + currentTargetAngle + " deg");
    }

    @Override
    public void execute() {
        ingestor.stopAll();
        drivetrain.swerveDrive(0.0, 0.0, 0.0, false);
        // shooter.setShooterRpm(0.0);
        climber.arm1Hold();
        autoShootTest();
    }

    private void autoShootTest() {
        if (isHolding) {
            shooter.setHoodSpeedPct(0.0);
            shooter.setShooterRpm(currentTargetRpm);
            if (commandTimer.get() >= 3.0) { // Three seconds elapsed. Move to the next step. Or end command if
                                             // encountered a failure.
                isHolding = false;
                String testRoutine = "TestAutoShoot ";
                if (encounteredFailure) {          
                    testRoutine += currentTargetRpm + " rpm, " + currentTargetAngle + " deg (FAILED)";          
                    tableIndex = rpmList.size() + 1;
                } else {
                    tableIndex++;
                    if (tableIndex < Math.min(rpmList.size(), angleList.size())) {
                        currentTargetRpm = rpmList.get(tableIndex).getSecond();
                        currentTargetAngle = angleList.get(tableIndex).getSecond();
                        testRoutine += currentTargetRpm + " rpm, " + currentTargetAngle + " deg";
                    } else {
                        testRoutine += "(FINISHED)";
                    }
                    commandTimer.reset();
                }
                SmartDashboard.putString("Current Test Routine", testRoutine);
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
        return tableIndex >= Math.min(angleList.size(), rpmList.size());
    }
}
