package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.*;
//import frc.robot.Snapshot;

public class AutoShoot extends CommandBase {
    private final Drivetrain drive;
    private final Shooter shooter;
    private final XboxController operatorController;
    private final XboxController driverController;
    private final Ingestor ingestor;
    private final CenterToHub centerToHub;
    private boolean cargoSentToShooter;
    // private boolean autonShootFinished;
    // private boolean lastShot;
    private boolean snapshotTaken;
    private boolean centerScheduled;

    public AutoShoot(Drivetrain drive, Shooter shooter, Ingestor ingestor, XboxController operatorController,
            XboxController driverController) {
        this.drive = drive;
        this.shooter = shooter;
        this.operatorController = operatorController;
        this.driverController = driverController;
        this.ingestor = ingestor;
        centerToHub = new CenterToHub(drive);
        cargoSentToShooter = false;
        // autonShootFinished = false;
        // lastShot = false;
        snapshotTaken = false;
        centerScheduled = false;

        addRequirements(drive, shooter);
    }

    @Override
    public void initialize() {
        cargoSentToShooter = false;
    }

    @Override
    public void execute() {
        shooter.calcShot(false);
        String error = "";

        // check hood angle is more than 3* off
        double targetHoodAngle = shooter.getTargetHoodAngle();
        shooter.setHoodAngle(targetHoodAngle);
        if (Math.abs(shooter.getHoodAngle() - targetHoodAngle) > 0.5) {
            error = String.join(error, "Hood ");
        }

        // check shot speed is within 30 RPM
        double targetRpm = shooter.getTargetRpm();
        shooter.setShooterRpm(targetRpm);
        if (Math.abs(shooter.getShooterVelocity() - targetRpm) > 30.0) {
            error = String.join(error, "RPM ");
        }

        // check if PI saw target (minimum shot distance 2.1-ish meters)
        if (Pi.getTargetCenterX() > 0.0) {
            if (operatorController != null && driverController != null) {
                Robot.stopControllerRumble(operatorController);
                Robot.stopControllerRumble(driverController);
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
            // error = String.join(error, "TurnL ");
            // // left is positive turn
            // drive.swerveDrive(0.0, 0.0, -rotationSpeed, false);
            // } else if (Pi.getTargetMoveRight()) {
            // error = String.join(error, "TurnR ");
            // drive.swerveDrive(0.0, 0.0, rotationSpeed, false);
            // } else {
            // // robot centered, stop driving
            // drive.swerveDrive(0.0, 0.0, 0.0, false);
            // }
        } else {
            // pi is not seeing hub
            if (operatorController != null && driverController != null) {
                Robot.rumbleController(operatorController, 1.0);
                Robot.rumbleController(driverController, 1.0);
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
            //System.out.println("You need to shoot!");
            if (ingestor.sendCargoToShooter()) { // sends cargo to shooter and returns true once it finishes sending
                // cargo
                cargoSentToShooter = true;
            }
            /*
             * if (!lastShot) {
             * // Snapshot.TakeSnapshot("SHOT");
             * }
             */
            // lastShot = true;
            SmartDashboard.putBoolean("auto shot shooting", true);
        } else {
            // lastShot = false;
            SmartDashboard.putBoolean("auto shot shooting", false);
        }
        SmartDashboard.putString("Auto Shoot Error", error);
        System.out.println("Auto Shoot Error: " + error);

        if (!snapshotTaken) {
            // Snapshot.TakeSnapshot("START");
            snapshotTaken = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (operatorController != null && driverController != null) {
            Robot.stopControllerRumble(operatorController);
            Robot.stopControllerRumble(driverController);
        }
        Shooter.setCoast(true);
        // System.out.println("AutoShoot end");
        cargoSentToShooter = false;
        centerScheduled = false;
    }

    @Override
    public boolean isFinished() {
        if (cargoSentToShooter) {
            System.out.println("AutoShoot is finished");
        }
        return cargoSentToShooter;
    }
}
