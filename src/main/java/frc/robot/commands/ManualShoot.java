package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ColorSensor;
import frc.robot.Ingestor;
import frc.robot.Shooter;
import frc.robot.ColorSensor.CargoColor;

public class ManualShoot extends CommandBase {
    private Shooter shooter;
    private Ingestor ingestor;
    private double speed;

    public ManualShoot(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        speed = 2300;
        addRequirements(shooter);
        SmartDashboard.putNumber("Target RPM", speed); // 2300 = sweet spot based on '2022 shooter speed table'
    }

    @Override
    public void execute() {
        //System.out.println("Current ALLIANCE color: " + DriverStation.getAlliance() + "\nCurrent MANUAL SHOOT color sensor value: " + ColorSensor.getCargoColor());
        if (ColorSensor.getCargoColor().toString().equalsIgnoreCase(Shooter.getAllianceString()) || ColorSensor.getCargoColor() == CargoColor.Unknown) {
            shooter.setShooterRpm(speed);
            shooter.setHoodAngle(31); // knob 2.5
    
            // if target rpm is within range (+- 50)
            if (speed - 50 < shooter.getShooterVelocity() && shooter.getShooterVelocity() < speed + 50) {
                ingestor.sendCargoToShooter();
            }
        } else {
            CommandScheduler.getInstance().cancel(this);
        }
    }    

    @Override
    public void end(boolean interrupted) {
        Shooter.setCoast(true);
    }
}
