package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class AutonThreeBall extends CommandBase {
    private Drivetrain drive;
    private Shooter shooter;
    private Ingestor ingestor;
    private Timer timer;
    private boolean sentToShooter;
    private Rotation2d startRotation;


    public AutonThreeBall(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        this.drive = drive;
        this.shooter = shooter;
        this.ingestor = ingestor;
        sentToShooter = false;
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
        startRotation = drive.getRotation();
        // drive.currentStep++;
    }

    private void stop() {
        drive.drive(0, 0, 0, true);
    }

    @Override
    public void execute() {
        // TODO decrease time parameter to speed up the robot
        // System.out.println(drive.currentStep);
        switch (drive.currentStep) { // drive.currentStep = 2;
            case 0: // prep for auton. Lower ingestor at 1.5 times normal speed
                drive.drive(0.0, 0.0, 0.0, false);
                ingestor.lowerIngestor(1.5);
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                if (timer.get() >= 0.5) {
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            case 1: // drive forward with ingestor lowered and ready to ingest
                drive.drive(-Drivetrain.kMaxSpeed / 4, 0.0, 0.0, false);
                ingestor.lowerIngestor(0.0);
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                if(timer.get() >= 3.0) {//|| ingestor.getStage1Proximity()){
                    drive.currentStep++;
                    timer.reset();
                }
                /*
                if (timer.get() >= 3.0) {
                    drive.currentStep++;
                    timer.reset();
                }
                */
                break;
            case 2:
                drive.drive(Drivetrain.kMaxSpeed / 4, 0, 0, false);
                ingestor.lowerIngestor(0.0);
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                if(timer.get() >= 1.0) {
                    drive.currentStep++;
                    timer.reset();
                }
            case 3: // turn to hub. TODO: Maybe add vision?
                drive.drive(0.0, 0.0, Math.PI, false);
                ingestor.threeBallAutonIngest();
                ingestor.liftIngestor();
                shooter.setShooterRpm(1000.0);
                if (Math.abs(drive.getRotation().getDegrees() - startRotation.getDegrees()) >= 180) {
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            case 4: // shoot 2 balls with hood angle set at 2.5 knobs (aka, manual shot)
                drive.drive(0, 0, 0, false);
                double speed = 2300.0;
                // TODO: Might be able to schedule AutoLShoot later.
                // TODO add in hood angle code when working
                ingestor.lowerIngestor(0);
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
            case 5:
                drive.drive(0.0 , 0.0, -Math.PI, false); // Turn to cargo 3.
                ingestor.threeBallAutonIngest();
                ingestor.lowerIngestor(1.5);
                shooter.setShooterRpm(1000);
                if (timer.get() >= 1.0/3.0) {
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            default:
                drive.drive(0, 0, 0, false);
                ingestor.lowerIngestor(0.0);
                ingestor.getStage1Conveyor().set(ControlMode.PercentOutput, 0.0);
                ingestor.getStage2Conveyor().set(ControlMode.PercentOutput, 0.0);
                ingestor.getIngestorWheels().set(ControlMode.PercentOutput, 0.0);
                shooter.setShooterRpm(1000.0);
                break;
        }
        // 8.586, 7.107836, 90 for first ball
        /*
         * drive.setPosition(8.586, 7.107836, Math.toRadians(90), 2, 1);
         * //TODO ingest
         * 
         * //turn 180
         * drive.setPosition(8.696, 7.0007836, Math.toRadians(270), 2, 2);
         * 
         * 
         * //TODO shoot
         * 
         * //drive to next ball: 10.340534, 6.433, -30 degrees
         * drive.setPosition(10.340534, 6.433, Math.toRadians(330), 2, 3);
         * 
         * //TODO ingest
         * 
         * //turn to 220 degrees
         * drive.setPosition(10.450534, 6.543, Math.toRadians(220), 2, 4);
         * 
         * 
         * //TODO shoot
         */

    }

    @Override
    public boolean isFinished() {
        return drive.currentStep == 6; // change this if any setPosition steps are added in execute()
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.drive(0, 0, 0, false);
        // shooter.setShooterRpm(1000.0);
        Shooter.setCoast(false);
        ingestor.lowerIngestor(0);
        ingestor.getStage1Conveyor().set(ControlMode.PercentOutput, 0);
        ingestor.getStage2Conveyor().set(ControlMode.PercentOutput, 0);
        ingestor.getIngestorWheels().set(ControlMode.PercentOutput, 0);
    }

}
