package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.driving.CenterToHub;
import frc.robot.subsystems.ControllerIO;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ingestor;
import frc.robot.subsystems.Pi;
import frc.robot.subsystems.Shooter;

public class HybridShootNoLidar extends ShootCommand {
    private final Ingestor ingestor;
    private final Drivetrain drive;
    private final CenterToHub centerToHub;
    private boolean centerScheduled;
    private final boolean isUsingControllers;
    private boolean cargoSentToShooter;

    public HybridShootNoLidar(Shooter shooter, Ingestor ingestor, Drivetrain drive, boolean isUsingControllers) {
        super(shooter);
        this.ingestor = ingestor;
        this.drive = drive;
        this.isUsingControllers = isUsingControllers;
        this.centerToHub = new CenterToHub(drive);
    }

    @Override
    public void execute() {
        shooter.calcShot(false);
        String error = "";
        // check hood angle is more than 3* off
        targetHoodAngle = shooter.getTargetHoodAngle();
        targetRpm = shooter.getTargetRpm();
        super.execute();

        double hoodAngle = shooter.getHoodAngle();
        double shooterVelocity = shooter.getShooterVelocity();

        if (Math.abs(hoodAngle - targetHoodAngle) > 0.5) {
            error = String.join(error, "Hood ");
        }
        // check shot speed is within 30 RPM
        if (Math.abs(shooterVelocity - targetRpm) > 30.0) {
            error = String.join(error, "RPM ");
        }

        // check if PI saw target
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
