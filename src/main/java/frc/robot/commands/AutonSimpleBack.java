package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class AutonSimpleBack extends CommandBase {
    private Drivetrain drive;
    private Shooter shooter;
    private Ingestor ingestor;
    private Timer timer;
    private boolean sentToShooter;
    //private Rotation2d startRotation;

    public AutonSimpleBack(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        this.drive = drive;
        this.shooter = shooter;
        this.ingestor = ingestor;

        timer = new Timer();
        addRequirements(drive, shooter, ingestor);
        drive.currentStep = 0;
    }

    @Override
    public void initialize() {
        // TODO: what happens if we start on the equivalent red side of the field?
        // drive.odometry.resetPosition(new Pose2d(8.586, 6.05, new Rotation2d()), new
        // Rotation2d());
        timer.reset();
        timer.start();
        //startRotation = drive.getPose().getRotation();
        // drive.currentStep++;
    }

    @Override
    public void execute() {
        // TODO decrease time parameter to speed up the robot
        // System.out.println(drive.currentStep);
        switch (drive.currentStep) { // drive.currentStep = 2;
            case 0: // prep for auton. TODO: Lower hood?
                drive.drive(0.0, 0.0, 0.0, false);
                ingestor.liftIngestor();
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                if (timer.get() >= 0.5) {
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            case 1: // drive backwards to shoot. TODO: Raise hood to
                    // manual shot angle?
                // negative x value for drive is forward, positive x val is backwards because motors are currently inverted
                drive.drive(Drivetrain.kMaxSpeed / 4, 0, 0.0, false);
                ingestor.liftIngestor();
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                if (timer.get() >= 3.0) {// || ingestor.getStage1Proximity()){
                    drive.currentStep++;
                    timer.reset();
                }
                /*
                 * if (timer.get() >= 3.0) {
                 * drive.currentStep++;
                 * timer.reset();
                 * }
                 */
                break;
            case 2: // shoot one preloaded ball.
                drive.drive(0.0, 0.0, 0.0, false);
                double speed = 2300.0;
                // TODO: Might be able to schedule AutoLShoot later.
                // TODO add in hood angle code when working
                ingestor.liftIngestor();
                shooter.setShooterRpm(speed);
                if (speed - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < speed + 50) {
                    ingestor.sendCargoToShooter();
                    sentToShooter = true;
                    timer.reset();
                }
                if (timer.get() >= 2.0 && sentToShooter) {
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            default:
                drive.drive(0.0, 0.0, 0.0, false);
                ingestor.liftIngestor();
                ingestor.getStage1Conveyor().set(ControlMode.PercentOutput, 0.0);
                ingestor.getStage2Conveyor().set(ControlMode.PercentOutput, 0.0);
                ingestor.getIngestorWheels().set(ControlMode.PercentOutput, 0.0);
                shooter.setShooterRpm(1000.0);
                break;
            /*
            case 3: // turn to hub. TODO: Maybe add vision?
                drive.drive(0.0, 0.0, Math.PI, false);
                ingestor.threeBallAutonIngest();
                ingestor.liftIngestor();
                shooter.setShooterRpm(1000.0);
                double angleDifference = Math.abs(drive.getRotation().getDegrees() - startRotation.getDegrees());
                if (angleDifference >= 180.0) {
                    drive.currentStep++;
                    timer.reset();
                } else {
                    System.out.println("Current angle difference: " + angleDifference + " degrees");
                }
                break;
            case 4: // shoot 2 balls with hood angle set at 2.5 knobs (aka, manual shot)
                drive.drive(0.0, 0.0, 0.0, false);
                double speed = 2300.0;
                // TODO: Might be able to schedule AutoLShoot later.
                // TODO add in hood angle code when working
                ingestor.liftIngestor();
                shooter.setShooterRpm(speed);
                if (speed - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < speed + 50) {
                    ingestor.sendCargoToShooter();
                    sentToShooter = true;
                    timer.reset();
                }
                if (timer.get() >= 2.0 && sentToShooter) {
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            */
            
        }

    }

    @Override
    public boolean isFinished() {
        return drive.currentStep == 3; // change this if any setPosition steps are added in execute()
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.drive(0, 0, 0, false);
        shooter.setShooterRpm(1000.0);
        Shooter.setCoast(true);
        ingestor.liftIngestor();
        ingestor.getStage1Conveyor().set(ControlMode.PercentOutput, 0);
        ingestor.getStage2Conveyor().set(ControlMode.PercentOutput, 0);
        ingestor.getIngestorWheels().set(ControlMode.PercentOutput, 0);
    }

}