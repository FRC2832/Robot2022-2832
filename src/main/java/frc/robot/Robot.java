// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


import frc.robot.SwerveConstants;
import frc.robot.SwerveModule;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutonOption0;
import frc.robot.commands.AutonOption1;
import frc.robot.commands.AutonOption2;
import frc.robot.commands.AutonOption3;
import frc.robot.commands.AutonOption4;
import frc.robot.commands.AutonOption5;
import frc.robot.commands.AutonOption6;
import frc.robot.commands.DriveStick;
import frc.robot.commands.DriveStickSlew;
import frc.robot.commands.FourBallTerminal;
import frc.robot.commands.ResetOrientation;

public class Robot extends TimedRobot {
    private final XboxController controller = new XboxController(0);
    private final Drivetrain swerve = new Drivetrain();
    private boolean lastEnabled = false;
    private AutonOption6 autonOption6;
    private AutonOption5 autonOption5;
    private AutonOption4 autonOption4;
    private AutonOption3 autonOption3;
    private AutonOption2 autonOption2;
    private AutonOption1 autonOption1;
    private AutonOption0 autonOption0;
    private FourBallTerminal fourBallTerminal;
    /*private Command auton5;
    private Command auton4;
    private Command auton3;
    private Command auton2;
    private Command auton1;*/
    private static final String auton1 = "Auton 1";
    private static final String auton2 = "Auton 2";
    private static final String auton3 = "Auton 3";
    private static final String auton4 = "Auton 4";
    private static final String auton5 = "Auton 5";
    SendableChooser<String> m_chooser = new SendableChooser<>();
    Odometry odometry;
    private String m_selectedAuton;



    @Override
    public void robotInit() {
        CommandScheduler.getInstance().registerSubsystem(swerve);
        swerve.setDefaultCommand(new DriveStickSlew(swerve, controller));
        // this.setNetworkTablesFlushEnabled(true); //turn off 20ms Dashboard update
        // rate
        LiveWindow.setEnabled(false);
        // add commands to the dashboard so we can run them seperately
        SmartDashboard.putData("Stick Drive", new DriveStick(swerve, controller));
        SmartDashboard.putData("Drive Forward 0.5mps", new AutoDrive(swerve, 0.5, 0));
        SmartDashboard.putData("Drive FR 0.5mps", new AutoDrive(swerve, 0.5, 0.5));
        SmartDashboard.putData("Reset Orientation", new ResetOrientation(swerve));

        /*m_chooser.setDefaultOption("Auton1", auton1);
        m_chooser.addOption("Auton2", auton2);
        m_chooser.addOption("Auton3", auton3);
        m_chooser.addOption("Auton4", auton4);
        m_chooser.addOption("Auton5", auton5);
        SmartDashboard.putData(m_chooser);*/
    }

    @Override
    public void disabledInit() {
        swerve.resetRobot();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        
       // m_selectedAuton = m_chooser.getSelected();
       // System.out.println("Auton Selected: " + m_selectedAuton);
       // CommandScheduler.getInstance().schedule(new AutonOption0(swerve));
        //CommandScheduler.getInstance().schedule(new AutonOption4(swerve));
        //CommandScheduler.getInstance().schedule(new AutonOption2(swerve));
        //CommandScheduler.getInstance().schedule(new AutonOption3(swerve));
       // Pose2d pos = swerve.odometry.getPoseMeters();
        
       
    }

    @Override
    public void autonomousPeriodic(){
       CommandScheduler.getInstance().schedule(new FourBallTerminal(swerve));
       // Pose2d pos = swerve.odometry.getPoseMeters();
           // swerve.setPosition(10.85, 6.13, 0, 2);
           // swerve.setPosition(8.57, 7.53, 0, 1);
            
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // have the field position constantly update
        swerve.updateOdometry();
        //SmartDashboard.putNumber("XPosition", odometry.getXPosition());
       // SmartDashboard.putNumber("YPosition", odometry.getYPosition());

        // automatically turn on/off recording
        if (lastEnabled != isEnabled()) {
            // we know the enabled status changed
            if (lastEnabled == false) {
                // robot started, start recording
                Shuffleboard.startRecording();
            } else {
                // robot stopped, stop recording
                Shuffleboard.stopRecording();
            }
        }
        // save the result for next loop
        lastEnabled = isEnabled();

        SmartDashboard.putData(swerve);
    }
}
