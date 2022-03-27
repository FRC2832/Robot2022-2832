package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Pi;
import frc.robot.Shooter;

public class AutonThreeBall extends CommandBase {
    private Drivetrain drive;
    private Shooter shooter;
    private Ingestor ingestor;
    private Timer timer;
    private boolean sentToShooter;
    private Rotation2d startRotation;
    private double startAngle;
    private double startEncoderCount;
    private AutoShoot autoShoot;
    private static boolean scheduled;

    public AutonThreeBall(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        this.drive = drive;
        this.shooter = shooter;
        this.ingestor = ingestor;
        autoShoot = new AutoShoot(drive, shooter, ingestor, null, null);
        scheduled = false;

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
        startAngle = drive.getAngle() % 360;
        startEncoderCount = drive.getModules()[0].getDistance();
        //System.out.println("Start X, Y: " + startPose.getX() + ", " + startPose.getY());
        System.out.println("Start encoder distance value: " + startEncoderCount);
        System.out.println("Start angle: " + startAngle + " degrees");
        
        // drive.currentStep++;
    }

    @Override
    public void execute() {
        double distance;
        // TODO decrease time parameter to speed up the robot
        // System.out.println(drive.currentStep);
        switch (drive.currentStep) { // drive.currentStep = 2;
            case 0:
                if(Pi.getCargoMoveLeft()) {
                    drive.drive(0, 0, -Math.toRadians(70), false);
                } else if(Pi.getCargoMoveRight()) {
                    drive.drive(0, 0, Math.toRadians(70), false);
                } else {
                    drive.drive(0, 0, 0, false);
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            case 1: // prep for auton. Lower ingestor at 1.5 times normal speed TODO: Lower hood?
                drive.drive(0.0, 0.0, 0.0, false);
                ingestor.lowerIngestor(1.5);
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                if (timer.get() >= 0.5) {
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            case 2: // drive forward with ingestor lowered and ready to ingest. TODO: Raise hood to
                    // manual shot angle?
                // negative x value for drive because motors are currently inverted
                drive.drive(-Drivetrain.kMaxSpeed / 4, 0, 0.0, false);
                ingestor.lowerIngestor(0.0);
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                distance = Math.abs(drive.getModules()[0].getDistance() - startEncoderCount);
                if (distance >= 1.5) {//timer.get() >= 3.0) {// || ingestor.getStage1Proximity()){
                    drive.currentStep++;
                    timer.reset();
                    startEncoderCount = drive.getModules()[0].getDistance();
                } else {
                    System.out.println("Current distance (step 2): " + distance);
                }
                /*
                 * if (timer.get() >= 3.0) {
                 * drive.currentStep++;
                 * timer.reset();
                 * }
                 */
                break;
            case 3: // back off so there's room to turn around.
                drive.drive(Drivetrain.kMaxSpeed / 4, 0.0, 0.0, false);
                ingestor.lowerIngestor(0.0);
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                distance = Math.abs(drive.getModules()[0].getDistance() - startEncoderCount);
                if (distance >= 0.5) {
                    drive.currentStep++;
                    timer.reset();
                    //nextStepStartPose = drive.getPose();
                } else {
                    System.out.println("Current distance (step 3): " + distance);
                }
                break;
            case 4: // turn to hub. TODO: Maybe add vision?
                drive.drive(0.0, 0.0, Math.PI, false);
                ingestor.threeBallAutonIngest();
                ingestor.liftIngestor();
                shooter.setShooterRpm(1000.0);
                double angleDifference = Math.abs(drive.getAngle() % 360 - startAngle);
                if (angleDifference >= 180.0) {
                    drive.currentStep++;
                    timer.reset();
                } else {
                    System.out.println("Current angle difference: " + angleDifference + " degrees");
                }
                break;
            case 5: // shoot 2 balls with hood angle set at 2.5 knobs (aka, manual shot)
                drive.drive(0.0, 0.0, 0.0, false);
                // double speed = 2300.0;
                // // TODO: Might be able to schedule AutoLShoot later.
                // // TODO add in hood angle code when working
                // ingestor.lowerIngestor(0.0);
                // shooter.setShooterRpm(speed);
                // if (speed - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < speed + 50) {
                //     ingestor.sendCargoToShooter();
                //     sentToShooter = true;
                //     timer.reset();
                // }
                if (!scheduled) {
                    CommandScheduler.getInstance().schedule(autoShoot);
                    scheduled = true;
                }
                if (autoShoot.isFinished()) {
                    drive.currentStep++;
                }
                break;
            /*
            case 5:
                drive.drive(0.0, 0.0, -Math.PI, false); // Turn to cargo 3.
                ingestor.threeBallAutonIngest();
                ingestor.lowerIngestor(1.5);
                shooter.setShooterRpm(1000);
                if (timer.get() >= 1.0 / 3.0) {
                    drive.currentStep++;
                    timer.reset();
                }
                break;
            
            case 6:
                drive.drive(-Drivetrain.kMaxSpeed / 4, 0, 0.0, false); // Move towards cargo 3
                ingestor.lowerIngestor(0.0);
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(1000.0);
                if (timer.get() >= 4.5) {// || ingestor.getStage1Proximity()){
                    drive.currentStep++;
                    timer.reset();
                }
                /*
                break;
            case 7:
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
                
            */
        }

    }

    @Override
    public boolean isFinished() {
        return drive.currentStep == 6; // change this if any setPosition steps are added in execute()
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.drive(0, 0, 0, false);
        shooter.setShooterRpm(1000.0);
        Shooter.setCoast(true);
        ingestor.lowerIngestor(0);
        ingestor.getStage1Conveyor().set(ControlMode.PercentOutput, 0);
        ingestor.getStage2Conveyor().set(ControlMode.PercentOutput, 0);
        ingestor.getIngestorWheels().set(ControlMode.PercentOutput, 0);
    }

    public static void resetAutonShoot() {
        scheduled = false;
    }

}
