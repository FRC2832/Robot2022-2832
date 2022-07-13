package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Shooter;
import frc.robot.SwerveModule;

public class AutonAlt extends CommandBase {
    private final Timer timer;
    private final Drivetrain drive;
    private final Shooter shooter;
    private final Ingestor ingestor;
    private final SwerveModule frontLeft;
    private double startAngle;
    private double startEncoderCount;
    
    public AutonAlt(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        timer = new Timer();
        this.drive = drive;
        this.shooter = shooter;
        this.ingestor = ingestor;
        frontLeft = drive.getModules()[0];
        addRequirements(drive, shooter, ingestor);
        drive.resetCurrentStep();
    }

    @Override
    public void initialize() {
        drive.resetCurrentStep();
        startAngle = drive.getAngle() % 360;
        startEncoderCount = frontLeft.getDistance();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double distance;
        double angle = 35.0;
        // TODO decrease time parameter to speed up the robot
        // System.out.println(drive.currentStep);
        switch (drive.getCurrentStep()) { // drive.currentStep = 2;
            case 0: // prep for auton. Lower ingestor at 1.5 times normal speed TODO: Lower hood?
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(2300.0);
                shooter.setHoodAngle(angle);
                if (timer.get() >= 0.5) {
                    drive.incrementCurrentStep();
                    timer.reset();
                }
                break;
            case 1: // drive forward with ingestor lowered and ready to ingest. TODO: Raise hood to
                // manual shot angle?
                // negative x value for drive because motors are currently inverted
                drive.swerveDrive(-Drivetrain.kMaxSpeed / 3, 0.0, 0.0, false);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(2300.0);
                shooter.setHoodAngle(angle);
                distance = Math.abs(frontLeft.getDistance() - startEncoderCount);
                if (distance >= 1.5) {// timer.get() >= 3.0) {// || ingestor.getStage1Proximity()){
                    drive.incrementCurrentStep();
                    if (SmartDashboard.getBoolean("Skip Reverse Auton Drive", false)) {
                        drive.incrementCurrentStep();
                    }
                    timer.reset();
                    startEncoderCount = frontLeft.getDistance();
                }
                break;
            case 2: // back off so there's room to turn around.
                drive.swerveDrive(Drivetrain.kMaxSpeed / 3, 0.0, 0.0, false);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(2300.0);
                shooter.setHoodAngle(angle);
                distance = Math.abs(frontLeft.getDistance() - startEncoderCount);
                if (distance >= 0.5) {
                    drive.incrementCurrentStep();
                    timer.reset();
                } 
                break;
            case 3: // turn to hub. 
                drive.swerveDrive(0.0, 0.0, (3 * Math.PI) / 4, false);
                ingestor.threeBallAutonIngest();
                ingestor.liftIngestor();
                shooter.setShooterRpm(2300.0);
                shooter.setHoodAngle(angle);
                double angleDifference = Math.abs(drive.getAngle() % 360 - startAngle);
                if (angleDifference >= 155.0) {
                    drive.swerveDrive(0.0, 0.0, 0.0, false);
                    drive.incrementCurrentStep();
                    timer.reset();
                }
                break;
            case 4: // shoot 2 balls with hood angle set at 2.5 knobs (aka, manual shot)
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                double speed = 2370.0;
                ingestor.liftIngestor();
                shooter.setShooterRpm(speed);
                shooter.setHoodAngle(angle);
                if (speed - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < speed + 50) {
                    if (ingestor.sendCargoToShooter()) {
                        timer.reset();
                        drive.incrementCurrentStep();
                    }
                }
                break;
            default:
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                shooter.setShooterRpm(2300.0);
                ingestor.liftIngestor();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.swerveDrive(0.0, 0.0, 0.0, false);
        shooter.setShooterRpm(2300.0);
        Shooter.setCoast(true);
        ingestor.liftIngestor();
        ingestor.getStage1Conveyor().set(ControlMode.PercentOutput, 0.0);
        ingestor.getStage2Conveyor().set(ControlMode.PercentOutput, 0.0);
        ingestor.getIngestorWheels().set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean isFinished() {
        return drive.getCurrentStep() == 5; // change this if any setPosition steps are added in execute()
    }
}
