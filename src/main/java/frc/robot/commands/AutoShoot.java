package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.*;

public class AutoShoot extends CommandBase {
    private Drivetrain drive;
    private Shooter shooter;
    private Pi pi;
    private XboxController controller;
    private Ingestor ingestor;

    public AutoShoot(Drivetrain drive, Shooter shooter, Ingestor ingestor, Pi pi, XboxController controller) {
        this.drive = drive;
        this.shooter = shooter;
        this.pi = pi;
        this.controller = controller;
        this.ingestor = ingestor;

        addRequirements(drive);
        addRequirements(shooter);
    }

    public void execute() {
        shooter.calcShot();
        String error = "";

        // check hood angle is more than 3* off
        // shooter.setHoodAngle(shooter.getTargetHoodAngle());
        if (!shooter.isHoodHomed() || Math.abs(shooter.getHoodAngle() - shooter.getTargetHoodAngle()) > 3) {
            // TODO: turned off hood since it's broke
            error = String.join(error, "Hood ");
        }
        // check shot speed is within 30 RPM
        shooter.setShooterRpm(shooter.getTargetRpm());
        if (Math.abs(shooter.getShooterVelocity() - shooter.getTargetRpm()) > 30) {
            error = String.join(error, "RPM ");
        }

        // check if PI saw target
        if (pi.getCenterX() > 0) {
            controller.setRumble(RumbleType.kLeftRumble, 0.0);
            controller.setRumble(RumbleType.kRightRumble, 0.0);
            if (Pi.getTargetMoveLeft()) {
                error = String.join(error, "TurnL ");
                // left is positive turn
                drive.drive(0, 0, Math.toRadians(70), false);
            } else if (Pi.getTargetMoveRight()) {
                error = String.join(error, "TurnR ");
                drive.drive(0, 0, -Math.toRadians(70), false);
            } else {
                // robot centered, stop driving
                drive.drive(0, 0, 0, false);
            }
        } else {
            // pi is not seeing hub
            controller.setRumble(RumbleType.kLeftRumble, 1.0);
            controller.setRumble(RumbleType.kRightRumble, 1.0);
            error = String.join(error, "Vision ");
            drive.drive(0, 0, 0, false);
        }

        // check for driving (0.15m/s == 6in/s)
        if (Math.abs(drive.getModules()[0].getVelocity()) > 0.15) {
            error = String.join(error, "Driving ");
            // driving might be because of centering, so don't stop it
        }

        if (error.length() == 0) {
            ingestor.sendCargoToShooter();
            error = "SHOOT!!!";
        }
        SmartDashboard.putString("Auto Shoot Error", error);
    }

    public void end() {
        controller.setRumble(RumbleType.kLeftRumble, 0.0);
        controller.setRumble(RumbleType.kRightRumble, 0.0);
    }
}
