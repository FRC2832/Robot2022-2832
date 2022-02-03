// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.Auton1;
import frc.robot.Commands.Auton1A;
import frc.robot.Commands.DriveStick;
import frc.robot.Commands.DriveStickSlew;
import frc.robot.Commands.ResetOrientation;

public class Robot extends TimedRobot {
    private final XboxController controller = new XboxController(0);
    private final Drivetrain swerve = new Drivetrain();
    private boolean lastEnabled = false;

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().registerSubsystem(swerve);
        swerve.setDefaultCommand(new DriveStickSlew(swerve,controller));
        //this.setNetworkTablesFlushEnabled(true);  //turn off 20ms Dashboard update rate
        LiveWindow.setEnabled(false);
        Pi.sendAlliance();

        //add commands to the dashboard so we can run them seperately
        SmartDashboard.putData("Stick Drive", new DriveStick(swerve, controller));
        SmartDashboard.putData("Drive Forward 0.5mps", new AutoDrive(swerve, 0.5, 0));
        SmartDashboard.putData("Drive FR 0.5mps", new AutoDrive(swerve, 0.5, 0.5));
        SmartDashboard.putData("Reset Orientation", new ResetOrientation(swerve));
        SmartDashboard.putNumber("Shooting delay", 0.0);
    }

    @Override
    public void disabledInit() {
        swerve.resetRobot();
    }
    
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(new Auton1(swerve));
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopPeriodic() {

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
