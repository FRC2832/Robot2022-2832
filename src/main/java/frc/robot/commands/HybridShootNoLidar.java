package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Pi;
import frc.robot.Robot;
import frc.robot.Shooter;

public class HybridShootNoLidar extends CommandBase {
    private final Shooter shooter;
    private final Ingestor ingestor;
    private final Drivetrain drive;
    private final XboxController operatorController;
    private final XboxController driverController;
    private final CenterToHub centerToHub;
    private boolean centerScheduled;
    private boolean isUsingControllers;
    private boolean cargoSentToShooter;

    public HybridShootNoLidar(Shooter shooter, Ingestor ingestor, Drivetrain drive, XboxController operatorController,
            XboxController driverController) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        this.drive = drive;
        this.operatorController = operatorController;
        this.driverController = driverController;
        isUsingControllers = operatorController != null && driverController != null;
        this.centerToHub = new CenterToHub(drive);
        addRequirements(drive, shooter);
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

        // check if PI saw target
        if (Pi.getTargetCenterX() > 0.0) {
            if (isUsingControllers) {
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
            if (isUsingControllers) {
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
            // System.out.println("You need to shoot!");
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
            SmartDashboard.putBoolean("hybrid shot shooting", true);
        } else {
            // lastShot = false;
            SmartDashboard.putBoolean("hybrid shot shooting", false);
        }
        SmartDashboard.putString("Hybrid Shoot Error", error);
        // System.out.println("Auto Shoot Error: " + error);
    }

    @Override
    public boolean isFinished() {
        // System.out.println("AutoShoot is finished");
        return cargoSentToShooter;
    }
}
