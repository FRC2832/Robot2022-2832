package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Shooter;
import frc.robot.SwerveModule;

public class AutonTwoBall extends CommandBase {
    private final Timer timer;
    private static boolean isAutoShootScheduled;
    private final Drivetrain drive;
    private final Shooter shooter;
    private final Ingestor ingestor;
    private final AutoShoot autoShoot;
    private final SwerveModule frontLeft;
    // private boolean sentToShooter;
    private double startAngle;
    private double startEncoderCount;

    public AutonTwoBall(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        timer = new Timer();
        this.drive = drive;
        this.shooter = shooter;
        this.ingestor = ingestor;
        frontLeft = drive.getModules()[0];
        autoShoot = new AutoShoot(drive, shooter, ingestor, null, null);
        isAutoShootScheduled = false;
        addRequirements(drive, shooter, ingestor);
        drive.resetCurrentStep();
    }

    public static void resetAutonShoot() {
        isAutoShootScheduled = false;
    }

    @Override
    public void initialize() {
        // TODO: what happens if we start on the equivalent red side of the field?
        // drive.odometry.resetPosition(new Pose2d(8.586, 6.05, new Rotation2d()), new
        // Rotation2d());
        drive.resetCurrentStep();
        startAngle = drive.getAngle();
        startEncoderCount = frontLeft.getDistance();
        isAutoShootScheduled = false;
        timer.reset();
        timer.start();
        // System.out.println("Start X, Y: " + startPose.getX() + ", " +
        // startPose.getY());
        // System.out.println("Start encoder distance value: " + startEncoderCount);
        // System.out.println("Start angle: " + startAngle + " degrees");

        // drive.currentStep++;
    }

    @Override
    public void execute() {
        double distance;
        // TODO decrease time parameter to speed up the robot
        // System.out.println(drive.currentStep);
        switch (drive.getCurrentStep()) { // drive.currentStep = 2;
            case 0: // prep for auton. Lower ingestor at 1.5 times normal speed TODO: Lower hood?
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(2300.0);
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
                distance = Math.abs(frontLeft.getDistance() - startEncoderCount);
                if (distance >= 1.5) {// timer.get() >= 3.0) {// || ingestor.getStage1Proximity()){
                    drive.incrementCurrentStep();
                    if (SmartDashboard.getBoolean("Skip Reverse Auton Drive", false)) {
                        drive.incrementCurrentStep();
                    }
                    timer.reset();
                    startEncoderCount = frontLeft.getDistance();
                } /*
                   * else {
                   * //System.out.println("Current distance (step 2): " + distance);
                   * }
                   */
                /*
                 * if (timer.get() >= 3.0) {
                 * drive.currentStep++;
                 * timer.reset();
                 * }
                 */
                break;
            case 2: // back off so there's room to turn around.
                drive.swerveDrive(Drivetrain.kMaxSpeed / 3, 0.0, 0.0, false);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                shooter.setShooterRpm(2300.0);
                distance = Math.abs(frontLeft.getDistance() - startEncoderCount);
                if (distance >= 0.5) {
                    drive.incrementCurrentStep();
                    timer.reset();
                    //during the first 2 steps, the robot drifts sideways, reset zero angle so that the turn is better
                    startAngle = drive.getAngle();
                    // nextStepStartPose = drive.getPose();
                } /*
                   * else {
                   * System.out.println("Current distance (step 3): " + distance);
                   * }
                   */
                break;
            case 3: // turn to hub. TODO: Maybe add vision?
                drive.swerveDrive(0.0, 0.0, (3 * Math.PI) / 4, false);
                ingestor.threeBallAutonIngest();
                ingestor.liftIngestor();
                shooter.setShooterRpm(2300.0);
                double angleDifference = Math.abs(drive.getAngle() - startAngle);
                if (angleDifference >= 170.0) {
                    drive.incrementCurrentStep();
                    timer.reset();
                } /*
                   * else {
                   * System.out.println("Current angle difference: " + angleDifference +
                   * " degrees");
                   * }
                   */
                break;
            case 4: // shoot 2 balls with hood angle set at 2.5 knobs (aka, manual shot)
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                // double speed = 2300.0;
                // // TODO: Might be able to schedule AutoLShoot later.
                // // TODO add in hood angle code when working
                // ingestor.lowerIngestor(0.0);
                // shooter.setShooterRpm(speed);
                // if (speed - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity()
                // < speed + 50) {
                // ingestor.sendCargoToShooter();
                // sentToShooter = true;
                // TIMER.reset();
                // }
                if (!isAutoShootScheduled) {
                    CommandScheduler.getInstance().schedule(autoShoot);
                    isAutoShootScheduled = true;
                }
                if (autoShoot.isFinished()) {
                    drive.incrementCurrentStep();
                }
                break;
            default:
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                shooter.setShooterRpm(2300.0);
                ingestor.liftIngestor();
                break;
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
