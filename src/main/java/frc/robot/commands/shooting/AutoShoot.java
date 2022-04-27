package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.commands.driving.CenterToHub;
import frc.robot.subsystems.ControllerIO;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Pi;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends ShootCommand {
    private final Drivetrain drive;
    private final Ingestor ingestor;
    private final CenterToHub centerToHub;
    private final boolean isUsingControllers;
    private boolean cargoSentToShooter;
    // private boolean autonShootFinished;
    private boolean lastShot;
    private boolean snapshotTaken;
    private boolean centerScheduled; // TODO: Might need to make this static for it to work properly.

    public AutoShoot(Drivetrain drive, Shooter shooter, Ingestor ingestor, boolean isUsingControllers) {
        super(shooter);
        this.drive = drive;
        this.isUsingControllers = isUsingControllers;
        this.ingestor = ingestor;
        centerToHub = new CenterToHub(drive);
        cargoSentToShooter = false;
        // autonShootFinished = false;
        lastShot = false;
        snapshotTaken = false;
        centerScheduled = false;
        // adding drive as a requirement messes with auton
    }

    @Override
    public void initialize() {
        cargoSentToShooter = false;
        // TODO: Maybe add centerScheduled = false? Probably depends on whether the same
        // AutoShoot object is scheduled > 1 time.
    }

    @Override
    public void end(boolean interrupted) {
        if (isUsingControllers) {
            ControllerIO.getInstance().stopAllRumble();
        }
        Shooter.setCoast(true);
        // System.out.println("AutoShoot end");
        cargoSentToShooter = false;
        centerScheduled = false;
    }

    @Override
    public boolean isFinished() {
        if (cargoSentToShooter) {
            // System.out.println("AutoShoot is finished");
            Robot.setIsAutoShootFinished(true);
        }
        return cargoSentToShooter;
    }

    @Override
    public void execute() {
        shooter.calcShot(false);
        super.execute();
        String error = "";

        // check hood angle is more than 3* off
        targetHoodAngle = shooter.getTargetHoodAngle();
        double hoodAngle = shooter.getHoodAngle();
        if (Math.abs(hoodAngle - targetHoodAngle) > 1.0) {
            error = String.join(error, "Hood ");
        }

        // check shot speed is within 30 RPM
        targetRpm = shooter.getTargetRpm();
        double shooterVelocity = shooter.getShooterVelocity();
        if (Math.abs(shooterVelocity - targetRpm) > 30.0) {
            error = String.join(error, "RPM ");
        }

        // check if PI saw target (minimum shot distance 2.1-ish meters)
        if (Pi.getTargetCenterX() > 0.0) {
            if (isUsingControllers) {
                ControllerIO.getInstance().stopAllRumble();
            }
            if (!centerScheduled) {
                CommandScheduler.getInstance().schedule(centerToHub);
                centerScheduled = true;
            }
            if (!centerToHub.isFinished()) {
                error = String.join(error, "Centering");
            }
            // double rotationSpeed = Math.toRadians(50.0);
            // if (Pi.getTargetMoveLeft()) {
            //     error = String.join(error, "TurnL ");
            //     // left is positive turn
            //     drive.swerveDrive(0.0, 0.0, -rotationSpeed, false);
            // } else if (Pi.getTargetMoveRight()) {
            //     error = String.join(error, "TurnR ");
            //     drive.swerveDrive(0.0, 0.0, rotationSpeed, false);
            // } else {
            //     // robot centered, stop driving
            //     drive.swerveDrive(0.0, 0.0, 0.0, false);
            // }
        } else {
            // pi is not seeing hub
            if (isUsingControllers) {
                ControllerIO instance = ControllerIO.getInstance();
                instance.rumbleDriveController(1.0);
                instance.rumbleOpController(1.0);
            }
            error = String.join(error, "Vision ");
            drive.swerveDrive(0.0, 0.0, 0.0, false);
        }

        // check for driving (0.15m/s == 6in/s)
        if (Math.abs(drive.getModules()[0].getVelocity()) > 0.15) {
            error = String.join(error, "Driving ");
            // driving might be because of centering, so don't stop it
        }

        if (error.isEmpty()) {
            // error = "SHOOT!!!";
            // System.out.println("You need to shoot!");
            if (ingestor.sendCargoToShooter()) { // sends cargo to shooter and returns true once it finishes sending
                // cargo
                cargoSentToShooter = true;
            }
            if (!lastShot) {
                // Snapshot.TakeSnapshot("SHOT");
            }
            lastShot = true;
            SmartDashboard.putBoolean("auto shot shooting", true);
        } else {
            lastShot = false;
            SmartDashboard.putBoolean("auto shot shooting", false);
        }
        SmartDashboard.putString("Auto Shoot Error", error);
        // System.out.println("Auto Shoot Error: " + error);

        if (!snapshotTaken) {
            // Snapshot.TakeSnapshot("START");
            snapshotTaken = true;
        }
    }
}
