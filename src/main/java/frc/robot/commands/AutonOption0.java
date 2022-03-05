package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;
import frc.robot.Shooter;
import frc.robot.Odometry;

public class AutonOption0 extends CommandBase {
   
   private Drivetrain drive;

   public AutonOption0(Drivetrain drive) {
      this.drive = drive;
   }

   public void execute() {

      if(drive.currentStep ==0){
         drive.odometry.resetPosition(new Pose2d(8.3, 5.7, new Rotation2d()), new Rotation2d());
         drive.currentStep++;       
      }
     // System.out.println(drive.currentStep);
      //to get redBalls[1]
      drive.setPosition(8.57, 7.53, Math.toRadians(90), 1, 1);
      drive.setPosition(8.57, 7.53, Math.toRadians(270), 1, 2);
      //ingest
      //confirm ingesting
      
      //to get redBalls[0]
      

      //to get redBalls[2]270
     // drive.setPosition(10.95, 2.24, 0, .5, 3);

      //to get redBalls[3]
      //drive.setPosition(14.4, 6.77, 0, .5, 4);

      //to get redBalls[4]
      //drive.setPosition(14.89, 5.85, 0, .5, 5);

      //to get redBalls[5]
     // drive.setPosition(8.75, 0.68, 0, .5, 6);
   }
}