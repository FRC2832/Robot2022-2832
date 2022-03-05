package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Drivetrain;
import frc.robot.Shooter;
import frc.robot.Odometry;

public class FourBallTerminal extends CommandBase {
   
   private Drivetrain drive;

   Rotation2d angle = Rotation2d.fromDegrees(200);

   public FourBallTerminal(Drivetrain drive) {
      this.drive = drive;
   }
   

   public void execute() {
        System.out.println(drive.currentStep);
      if(drive.currentStep ==0){
         drive.odometry.resetPosition(new Pose2d(6.6, 2.8, new Rotation2d()), new Rotation2d());
         drive.currentStep++;       
      }
    
      drive.setPosition(5.13, 2.06, Math.toRadians(200), .5, 1);
      drive.setPosition(5.13, 2.06, Math.toRadians(20), .5, 2);
      drive.setPosition(1.55, 1.43, Math.toRadians(225), .5, 3);
      drive.setPosition(2.8,2.6, Math.toRadians(15), .5, 4);

      
     



      
   }
}