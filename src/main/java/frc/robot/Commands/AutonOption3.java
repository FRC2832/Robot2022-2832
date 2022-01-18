package frc.robot.Commands;

import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonOption3 extends CommandBase {
    private Timer timer; 

    private Drivetrain drive;

    public AutonOption3(Drivetrain drive){
        timer = new Timer();
        this.drive = drive; 
        addRequirements(drive);
     timer.start();
    }
    
   
private void stop(){
    drive.drive(0, 0, 0, true);
}
public void execute(){
   
   // timer.start();
    if(timer.get()<2){
        System.out.println("Moving Back");
        drive.drive(-1, 0, 0, false);
        } 
       else if(timer.get()>2 && timer.get()<3){
        System.out.println("Stopping");
           stop(); 
        } 
        //Pick up ball
        //shoot times two
      else if(timer.get()>3 && timer.get()< 6){
        System.out.println("Moving Right");
         drive.drive(0, -1, 0, false);
        }else{
            System.out.println("Stopping");
            stop();
        }
     //  timer.stop();
     //  timer.reset();
        
    }
 
}
