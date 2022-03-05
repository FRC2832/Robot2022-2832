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
   
      drive.setPosition(8.57, 7.53, Math.toRadians(90), 1, 1);
      drive.setPosition(8.57, 7.53, Math.toRadians(270), 1, 2);
     
   }
}