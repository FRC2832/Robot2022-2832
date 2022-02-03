package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

public class AutonOption6 extends CommandBase {
    private Drivetrain drive;
    private Timer timer;

    public AutonOption6(Drivetrain drive){
        timer = new Timer();
        this.drive = drive; 
        addRequirements(drive);
        timer.start();
    }
    private void stop(){
        drive.drive(0, 0, 0, true);
    }

    public void execute(){
        if(timer.get() < 2 ){
            drive.drive(.75, 0, 0, false);
            System.out.println("Getting ball 1");
            
        } else if (timer.get()>2 && timer.get()<3){
            stop();
            System.out.println("SHOOTING!!!!");
        }
        //pick up ball and shoot x2
        else if(timer.get()>3 && timer.get()< 4){
            drive.drive(0, 0, -3*Math.PI/4, false);
        }
        else if(timer.get()>4 && timer.get()<7){
            drive.drive(1, 0, 0, false);
            System.out.println("Getting ball 2");
        }
        else if(timer.get()>8 && timer.get()< 9){
            drive.drive(0, 0, Math.PI/2.5, false);
        } else if(timer.get()>9 && timer.get()<12){
            drive.drive(1.5, 0, 0, false);
        }
        else if(timer.get()>12 && timer.get()<13 ){
            drive.drive(0, 0, Math.PI, false);
        } else if(timer.get()>13 && timer.get()<14){
            drive.drive(2, 0, 0, false);
        } else{
            stop();
            System.out.println("SHOOTING!!!!");
        }
    }
        //Now we shoot!!!!!
}
    

