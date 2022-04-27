package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SwerveModule;
import frc.robot.commands.driving.CenterToCargo;
import frc.robot.commands.shooting.AutoShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Pi;
import frc.robot.subsystems.Shooter;

public class ThreeBallAuton extends CommandBase {
    private static final Timer TIMER = new Timer();
    private final Drivetrain drive;
    private final Ingestor ingestor;
    private final SwerveModule frontLeft;
    private final AutoShoot autoShoot;
    private final CenterToCargo centerToCargo;
    // private boolean sentTToShooter;
    private boolean isAutoShootScheduled;
    private boolean isCargoScheduled;
    private double startEncoderCount;
    private double startAngle;

    public ThreeBallAuton(Drivetrain drive, Shooter shooter, Ingestor ingestor) {
        super();
        this.drive = drive;
        this.ingestor = ingestor;
        // sentToShooter = false;
        frontLeft = drive.getModules()[0];
        autoShoot = new AutoShoot(drive, shooter, ingestor, false);
        centerToCargo = new CenterToCargo(drive);
        addRequirements(drive, ingestor);
        drive.resetCurrentStep();
    }

    @Override
    public void initialize() {
        // TODO: what happens if we start on the equivalent red side of the field?
        // drive.odometry.resetPosition(new Pose2d(8.586, 6.05, new Rotation2d()), new
        // Rotation2d());
        startAngle = drive.getAngle() % 360;
        startEncoderCount = frontLeft.getDistance();
        isAutoShootScheduled = false;
        isCargoScheduled = false;
        TIMER.reset();
        TIMER.start();
        // System.out.println("Start X, Y: " + startPose.getX() + ", " +
        // startPose.getY());
        // System.out.println("Start encoder distance value: " + startEncoderCount);
        // System.out.println("Start angle: " + startAngle + " degrees");

        // drive.currentStep++;
    }

    @Override
    public void execute() {
        double distance;
        double angleDifference;
        // TODO decrease time parameter to speed up the robot
        // System.out.println(drive.currentStep);
        switch (drive.getCurrentStep()) { // drive.currentStep = 2;
            case 0: // prep for auton. Lower ingestor at 1.5 times normal speed TODO: Lower hood?
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                // shooter.setShooterRpm(2300.0);
                if (TIMER.get() >= 0.5) {
                    drive.incrementCurrentStep();
                    TIMER.reset();
                }
                break;
            case 1: // drive forward with ingestor lowered and ready to ingest. TODO: Raise hood to
                // manual shot angle?
                // negative x value for drive because motors are currently inverted
                drive.swerveDrive(-Drivetrain.kMaxSpeed / 3, 0.0, 0.0, false);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                // shooter.setShooterRpm(2300.0);
                distance = Math.abs(frontLeft.getDistance() - startEncoderCount);
                if (distance >= 1.5) {// timer.get() >= 3.0) {// || ingestor.getStage1Proximity()){
                    drive.incrementCurrentStep();
                    if (SmartDashboard.getBoolean("Skip Reverse Auton Drive", false)) {
                        drive.incrementCurrentStep();
                    }
                    TIMER.reset();
                    startEncoderCount = frontLeft.getDistance();
                } /*
                   * else {
                   * //System.out.println("Current distance (step 2): " + distance);
                   * }
                   */
                /*
                 * if (timer.get() >= 3.0) {
                 * drive.currentStep++;
                 * TIMER.reset();
                 * }
                 */
                break;
            case 2: // back off so there's room to turn around.
                drive.swerveDrive(Drivetrain.kMaxSpeed / 3, 0.0, 0.0, false);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                // shooter.setShooterRpm(2300.0);
                distance = Math.abs(frontLeft.getDistance() - startEncoderCount);
                if (distance >= 0.5) {
                    drive.incrementCurrentStep();
                    TIMER.reset();
                    // nextStepStartPose = drive.getPose();
                } /*
                   * else {
                   * System.out.println("Current distance (step 3): " + distance);
                   * }
                   */
                break;
            case 3: // turn to hub. TODO: Maybe add vision?
                drive.swerveDrive(0.0, 0.0, Math.PI / 2, false);
                ingestor.threeBallAutonIngest();
                ingestor.liftIngestor();
                // shooter.setShooterRpm(2300.0);
                angleDifference = Math.abs(drive.getAngle() % 360 - startAngle);
                if (angleDifference >= 170.0) {
                    drive.incrementCurrentStep();
                    TIMER.reset();
                } /*
                   * else {
                   * System.out.println("Current angle difference: " + angleDifference +
                   * " degrees");
                   * }
                   */
                break;
            case 4: // Shoot 2 balls using AutoShoot
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                ingestor.liftIngestor();
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
                    isAutoShootScheduled = false;
                    startAngle = drive.getAngle() % 360;
                    TIMER.reset();
                }
                break;
            case 5: // Rotate 90 degrees left.
                angleDifference = Math.abs(drive.getAngle() % 360 - startAngle);
                ingestor.liftIngestor();
                ingestor.threeBallAutonIngest();
                // shooter.setShooterRpm(2300.0);
                if (angleDifference >= 90.0) { // TODO: Might need to account for no seen cargo.
                    drive.incrementCurrentStep();
                    drive.swerveDrive(0.0, 0.0, 0.0, false);
                } else {
                    drive.swerveDrive(0.0, 0.0, -Math.PI / 2, false);
                }
                break;
            case 6: // Rotate towards third cargo.
                // shooter.setShooterRpm(2300.0);
                ingestor.liftIngestor();
                ingestor.threeBallAutonIngest();
                if (Pi.getCargoCenterX() > -1.0 && Pi.getCargoCenterY() > -1.0) {
                    if (!isCargoScheduled) {
                        CommandScheduler.getInstance().schedule(centerToCargo);
                        isCargoScheduled = true;
                    }
                    if (centerToCargo.isFinished()) {
                        TIMER.reset();
                        startEncoderCount = frontLeft.getDistance();
                        drive.incrementCurrentStep();
                    }
                    // byte speedMultiplier = 0;
                    // if (Pi.getCargoMoveLeft()) {
                    // speedMultiplier = -1;
                    // } else if (Pi.getCargoMoveRight()) {
                    // speedMultiplier = 1;
                    // } else {
                    // drive.incrementCurrentStep();
                    // startEncoderCount = frontLeft.getDistance();
                    // TIMER.reset();
                    // }
                    // drive.swerveDrive(0.0, 0.0, speedMultiplier * (Math.PI / 2), false);
                } else {
                    drive.swerveDrive(0.0, 0.0, 0.0, false); // TODO: May need to add more logic here.
                }
                break;
            case 7: // Move towards cargo and ingest.
                // shooter.setShooterRpm(2300.0);
                ingestor.lowerIngestor();
                ingestor.threeBallAutonIngest();
                distance = Math.abs(frontLeft.getDistance() - startEncoderCount);
                if (distance >= 2.0) {
                    drive.swerveDrive(0.0, 0.0, 0.0, false);
                    drive.incrementCurrentStep();
                    TIMER.reset();
                    startAngle = drive.getAngle() % 360;
                } else {
                    drive.swerveDrive(-Drivetrain.kMaxSpeed / 3, 0.0, 0.0, false);
                }
                break;
            case 8: // Rotate right towards hub.
                // shooter.setShooterRpm(2300.0);
                ingestor.liftIngestor();
                ingestor.threeBallAutonIngest();
                angleDifference = Math.abs(drive.getAngle() % 360 - startAngle);
                if (angleDifference >= 120.0) {
                    drive.swerveDrive(0.0, 0.0, 0.0, false);
                    drive.incrementCurrentStep();
                    TIMER.reset();
                } else {
                    drive.swerveDrive(0.0, 0.0, Math.PI / 2, false);
                }
                break;
            case 9: // Shoot 1 ball using AutoShoot
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                if (!isAutoShootScheduled) {
                    CommandScheduler.getInstance()
                            .schedule(autoShoot); // TODO: May need to set autoShoot to new AutoShoot again.
                    isAutoShootScheduled = true;
                }
                if (autoShoot.isFinished()) {
                    drive.incrementCurrentStep();
                    isAutoShootScheduled = false;
                    TIMER.reset();
                }
                break;
            default:
                drive.swerveDrive(0.0, 0.0, 0.0, false);
                // shooter.setShooterRpm(2300.0);
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
        TIMER.stop();
        drive.swerveDrive(0.0, 0.0, 0.0, false);
        // shooter.setShooterRpm(1000.0);
        Shooter.setCoast(true);
        ingestor.stopAll();
    }

    @Override
    public boolean isFinished() {
        return drive.getCurrentStep() == 10; // change this if any setPosition steps are added in execute()
    }

}
