package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;
import frc.robot.Shooter;
import frc.robot.Odometry;

public class AutonOption6 extends CommandBase {
   
   private Drivetrain drive;

   public AutonOption6(Drivetrain drive) {
      this.drive = drive;
   }

   public void execute() {

      if(drive.currentStep ==0){
         drive.odometry.resetPosition(new Pose2d(8.3, 5.7, new Rotation2d()), new Rotation2d());
         drive.currentStep++;       
      }
    
      drive.setPosition(8.57, 7.53, Math.toRadians(90), .5, 1);
      drive.setPosition(8.57, 7.53, Math.toRadians(270), .5, 2);
      drive.setPosition(10.85, 6.13, Math.toRadians(320), .5, 3);
      drive.setPosition(10.85, 6.13, Math.toRadians(220), .5, 4);
      drive.setPosition(14.4, 6.77, Math.toRadians(45), .5, 5);
      drive.setPosition(12.3, 5.3, Math.toRadians(200), .5, 6);



      
   }
}