package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Shooter;

public class AutonOption6 extends CommandBase {
    private Drivetrain driveTrain;
    private Shooter shooter;
    private Timer timer;

    public AutonOption6(Drivetrain driveTrain, Shooter shooter) {
        timer = new Timer();
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        addRequirements(driveTrain, shooter);
        timer.start();
    }

    private void stopDrive() {
        driveTrain.drive(0, 0, 0, true);
    }

    public void execute() {
        double timerVal = timer.get();
        if (timerVal < 2) {
            driveTrain.drive(1, 0, 0, false);
            shooter.setShooterRpm(0);
            System.out.println("Approaching cargo 2");
        } else if (timerVal < 3) {
            driveTrain.drive(0, 0, Math.PI, false);
            shooter.setShooterRpm(60);
            System.out.println("Turning to shoot");
        } else if (timerVal < 4) {
            shooter.setShooterRpm(60);
            stopDrive();
            System.out.println("Shooting two cargo");
        } else if (timerVal < 5) {
            shooter.setShooterRpm(0);
            driveTrain.drive(1.05, -1.819, Math.PI / 3, true);
            System.out.println("turning and driving towards 3rd cargo");
            // TODO: Replace with vision
        } else if (timerVal < 6) {
            shooter.setShooterRpm(60);
            driveTrain.drive(0, 0, (-2 * Math.PI) / 3, false);
            System.out.println("Turning to shoot cargo 3");
        } else if (timerVal < 7) {
            shooter.setShooterRpm(60);
            stopDrive();
            System.out.println("Shooting cargo 3");
        } else if (timerVal < 9) {
            shooter.setShooterRpm(0);
            driveTrain.drive(2, 0, Math.PI / 2, true);//-----
            System.out.println("Turning and driving towards cargo 4 and 5");
        } else if (timerVal < 10) {
            shooter.setShooterRpm(0);
            stopDrive();
            System.out.println("Stopping for a sec so the human player can deliver the cargo");
        } else if (timerVal < 11) {
            shooter.setShooterRpm(60);
            driveTrain.drive(-2, 0, Math.PI, true);
            System.out.println("Moving back and turning so the robot can shoot");
        } else if (timerVal < 12) {
            shooter.setShooterRpm(60);
            stopDrive();
            System.out.println("SHOOTING CARGO 4 AND 5!!!!");
        } else {
            shooter.setShooterRpm(0);
            stopDrive();
            System.out.println("STOPPING EVERYTHING!!!!!!!");
        }
    }
    // Now we shoot!!!!!
}
