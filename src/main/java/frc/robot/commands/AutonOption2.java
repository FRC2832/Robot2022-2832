package frc.robot.commands;

import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonOption2 extends CommandBase {
    private Timer timer;
    private Drivetrain drive;
    private double delay = 4.0;

    public AutonOption2(Drivetrain drive) {
        timer = new Timer();
        this.drive = drive;
        addRequirements(drive);
        delay = SmartDashboard.getNumber("Shooting Delay:", 0.0);
        timer.start();
    }

    private void stop() {
        drive.drive(0, 0, 0, true);
    }

    public void execute() {

        // timer.start();
        if (timer.get() < (2 + delay)) {
            System.out.println("Moving Back");
            drive.drive(-1, 0, 0, false);
        } 
        
        else if (timer.get() > 2 + delay && timer.get() < 3 + delay) {
            System.out.println("Stopping");
            stop();
        }
        // INGEST BALL
        // SHOOT TWICE

        else if(timer.get() > 3+delay && timer.get() < 4+delay){
            System.out.println("Turning Right to Face Ingestor to Ball");
            drive.drive(0, 0, -(Math.PI/2), false); //need to test third parameter: turn angle
        }

        else if (timer.get() > 4 + delay && timer.get() < 5 + delay) {
            System.out.println("Moving Toward Ball");
            drive.drive(-1, 0, 0, false);
        }

        //INGEST BALL PARTIALLY INTO INGESTOR

        else if(timer.get() > 5+delay && timer.get() < 6 + delay){
            System.out.println("Stopping");
            drive.drive(0, 0, 0, false);
        }

        else if(timer.get() > 6+delay && timer.get() < 7 + delay){
            System.out.println("Turning To Face Hangar");
            drive.drive(0, 0, 2, false); //need to test third parameter: turn angle
        }

        //SHOOT OUT OF INGESTOR INTO HANGAR WITH ENOUGH FORCE TO ROLL INTO HANGAR BUT NOT BOUNCE OUT

        else if(timer.get() > 7+delay && timer.get() < 8 + delay){
            System.out.println("Stopping");
            drive.drive(0, 0, 0, false);
        }

        else if (timer.get() > 8 + delay && timer.get() < 9 + delay) {
            System.out.println("Moving Forward Towards Center Line");
            drive.drive(1, 0, 0, false);
        }
        
        else if(timer.get() > 9+delay && timer.get() < 10 + delay){
            System.out.println("Turning To Face Center");
            drive.drive(0, 0, -2, false); //need to test third parameter: turn angle
        }
        
        else {
            System.out.println("Stopping");
            stop();
        }
        // timer.stop();
        // timer.reset();

    }

}