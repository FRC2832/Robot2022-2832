// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.DriveStick;
import frc.robot.Commands.DriveStickSlew;
import frc.robot.Commands.ResetOrientation;
import frc.robot.Commands.AutonOption3;
import frc.robot.Commands.AutonOption6;

public class Robot extends TimedRobot {
    private final XboxController controller = new XboxController(0);
    private final Drivetrain swerve = new Drivetrain();
    private boolean lastEnabled = false;
    private AutonOption3 autonOption3;

    private static final String option1 = "Option1";
  private static final String option2 = "Option2";
  private static final String option3 = "Option3";
  private static final String option4 = "Option4";
  private static final String option5 = "Option5";
  private static final String option6 = "Option6";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  DriveStickSlew driveStickSlew = new DriveStickSlew(swerve, controller);
    @Override
    public void robotInit() {
        CommandScheduler.getInstance().registerSubsystem(swerve);
<<<<<<< Updated upstream
        //swerve.setDefaultCommand(new DriveStickSlew(swerve,controller));
=======
        swerve.setDefaultCommand(new DriveStickSlew(swerve,controller));
>>>>>>> Stashed changes
        
        //this.setNetworkTablesFlushEnabled(true);  //turn off 20ms Dashboard update rate
        LiveWindow.setEnabled(false);

        //add commands to the dashboard so we can run them seperately
        SmartDashboard.putData("Stick Drive", new DriveStick(swerve, controller));
        SmartDashboard.putData("Drive Forward 0.5mps", new AutoDrive(swerve, 0.5, 0));
        SmartDashboard.putData("Drive FR 0.5mps", new AutoDrive(swerve, 0.5, 0.5));
        SmartDashboard.putData("Reset Orientation", new ResetOrientation(swerve));

        m_chooser.setDefaultOption("Option1", option1);
        m_chooser.addOption("Option2", option2);
        m_chooser.addOption("Option3", option3);
        m_chooser.addOption("Option4", option4);
        m_chooser.addOption("Option5", option5);
        m_chooser.addOption("Option6", option6);
        SmartDashboard.putData("Auto Choices", m_chooser);
    }

    @Override
    public void disabledInit() {
        swerve.resetRobot();
    }
    
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        m_autoSelected = m_chooser.getSelected();
      // autonOption3 = new AutonOption3(swerve);
        //CommandScheduler.getInstance().schedule(new AutonOption3(swerve));
       // CommandScheduler.getInstance().schedule(new AutonOption6(swerve));

        switch (m_autoSelected) {
            case option1:
            default:
              System.out.println("Running Option 1");
      
              break;
      
            case option2:
            System.out.println("Running Option 2");
              
              break;
      
              case option3:
              System.out.println("Running Option 3");
              CommandScheduler.getInstance().schedule(new AutonOption3(swerve));
              
              break;
      
            case option4:
            
            System.out.println("Running Option 4");
              
              break;

              case option5:
              System.out.println("Running Option 5");

              break;

              case option6:
              System.out.println("Running Option 6");
              CommandScheduler.getInstance().schedule(new AutonOption6(swerve));

              break;
          }
    }
    @Override
    public void autonomousPeriodic(){
        //autonOption3.execute();
       
    }

    @Override
    public void teleopPeriodic() {
        //swerve.drive(controller.getLeftY()*2.85, controller.getLeftX()*2.85, controller.getRightX()*6.28, true);
<<<<<<< Updated upstream
        driveStickSlew.execute();
=======
       // driveStickSlew.execute();
    }
    @Override 
    public void teleopInit(){
        CommandScheduler.getInstance().cancelAll();
>>>>>>> Stashed changes
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        //have the field position constantly update
        swerve.updateOdometry();

        //automatically turn on/off recording
        if(lastEnabled != isEnabled()) {
            //we know the enabled status changed
            if(lastEnabled == false) {
                //robot started, start recording
                Shuffleboard.startRecording();
            } else {
                //robot stopped, stop recording
                Shuffleboard.stopRecording();
            }
        }
        //save the result for next loop
        lastEnabled = isEnabled();

        SmartDashboard.putData(swerve);
    }
}
