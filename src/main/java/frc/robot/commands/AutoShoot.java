package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;
import frc.robot.Pi;
import frc.robot.Shooter;

public class AutoShoot extends CommandBase {
    private Drivetrain drive;
    private Shooter shooter;
    private XboxController operatorController;
    private XboxController driverController;
    private Ingestor ingestor;
    private boolean cargoSentToShooter;
    // private boolean autonShootFinished;

    public AutoShoot(Drivetrain drive, Shooter shooter, Ingestor ingestor, XboxController operatorController,
            XboxController driverController) {
        this.drive = drive;
        this.shooter = shooter;
        this.operatorController = operatorController;
        this.driverController = driverController;
        this.ingestor = ingestor;
        cargoSentToShooter = false;
        // autonShootFinished = false;

        addRequirements(drive);
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.calcShot();
        String error = "";

        // check hood angle is more than 3* off
        shooter.setHoodAngle(shooter.getTargetHoodAngle());
        if (Math.abs(shooter.getHoodAngle() - shooter.getTargetHoodAngle()) > 0.5) {
            error = String.join(error, "Hood ");
        }

        // check shot speed is within 30 RPM
        shooter.setShooterRpm(shooter.getTargetRpm());
        if (Math.abs(shooter.getShooterVelocity() - shooter.getTargetRpm()) > 30) {
            error = String.join(error, "RPM ");
        }

        // check if PI saw target
        if (Pi.getTargetCenterX() > 0) {
            if (operatorController != null && driverController != null) {
                operatorController.setRumble(RumbleType.kLeftRumble, 0.0);
                operatorController.setRumble(RumbleType.kRightRumble, 0.0);
                driverController.setRumble(RumbleType.kLeftRumble, 0.0);
                driverController.setRumble(RumbleType.kRightRumble, 0.0);
            }
            if (Pi.getTargetMoveLeft()) {
                error = String.join(error, "TurnL ");
                // left is positive turn
                drive.drive(0, 0, -Math.toRadians(70), false);
            } else if (Pi.getTargetMoveRight()) {
                error = String.join(error, "TurnR ");
                drive.drive(0, 0, Math.toRadians(70), false);
            } else {
                // robot centered, stop driving
                drive.drive(0, 0, 0, false);
            }
        } else {
            // pi is not seeing hub
            if (operatorController != null && driverController != null) {
                operatorController.setRumble(RumbleType.kLeftRumble, 1.0);
                operatorController.setRumble(RumbleType.kRightRumble, 1.0);
                driverController.setRumble(RumbleType.kLeftRumble, 1.0);
                driverController.setRumble(RumbleType.kRightRumble, 1.0);
            }
            error = String.join(error, "Vision ");
            drive.drive(0, 0, 0, false);
        }

        // check for driving (0.15m/s == 6in/s)
        if (Math.abs(drive.getModules()[0].getVelocity()) > 0.15) {
            error = String.join(error, "Driving ");
            // driving might be because of centering, so don't stop it
        }

        if (error.length() == 0) {
            // error = "SHOOT!!!";
            if (ingestor.sendCargoToShooter()) { // sends cargo to shooter and returns true once it finishes sending
                                                 // cargo
                cargoSentToShooter = true;
            }
        }
        SmartDashboard.putString("Auto Shoot Error", error);
    }

    @Override
    public boolean isFinished() {
        System.out.println("AutoShoot is finished");
        return cargoSentToShooter;
    }

    @Override
    public void end(boolean interrupted) {
        if (operatorController != null && driverController != null) {
            operatorController.setRumble(RumbleType.kLeftRumble, 0.0);
            operatorController.setRumble(RumbleType.kRightRumble, 0.0);
            driverController.setRumble(RumbleType.kLeftRumble, 0.0);
            driverController.setRumble(RumbleType.kRightRumble, 0.0);
        }
        Shooter.setCoast(true);
        System.out.println("AutoShoot end");
        cargoSentToShooter = false;
    }
}
