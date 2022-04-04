// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutonTwoBall;
//import frc.robot.commands.AutonTwoBall;
import frc.robot.commands.DriveStick;
import frc.robot.commands.DriveStickSlew;
import frc.robot.commands.LowerHubShoot;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.ResetOrientation;
import frc.robot.commands.RunClimber;
import frc.robot.commands.SafeZoneShoot;
import frc.robot.commands.ShooterOff;
import frc.robot.commands.UpperHubShoot;

public class Robot extends TimedRobot {
    private final XboxController DRIVER_CONTROLLER = new XboxController(0);
    private final XboxController OPERATOR_CONTROLLER = new XboxController(2);
    private Drivetrain swerve; // = new Drivetrain(DRIVER_CONTROLLER);
    private final Pi PI = new Pi();
    private final ColorSensor COLOR_SENSOR = new ColorSensor();
    private final Ingestor INGESTOR = new Ingestor(COLOR_SENSOR);
    private Shooter shooter;
    private Climber climber;
    private boolean ranAuton = false;
    //private TurtleMode turtleMode;

    private boolean lastEnabled = false;
    // Odometry odometry; // TODO: Can this be commented out?
    private String m_selectedAuton;

    /* private static final String option1 = "Option1";
    private static final String option2 = "Option2";
    private static final String option3 = "Option3";
    private static final String option4 = "Option4";
    private static final String option5 = "Option5";
    private static final String option6 = "Option6"; */
    //private String m_autoSelected;
    private final SendableChooser<String> CHOOSER = new SendableChooser<>();

    @Override
    public void robotInit() {
        swerve = new Drivetrain(DRIVER_CONTROLLER);
        Configuration.SetPersistentKeys();
        GitVersion vers = GitVersion.loadVersion();
        vers.printVersions();
        DataLogManager.start();

        ShooterConstants.LoadConstants();
        CommandScheduler.getInstance().registerSubsystem(PI);
        shooter = new Shooter(DRIVER_CONTROLLER, OPERATOR_CONTROLLER, INGESTOR);

        climber = new Climber(INGESTOR);

        //turtleMode = new TurtleMode(swerve, driverController);

        CommandScheduler.getInstance().registerSubsystem(swerve, COLOR_SENSOR);
        swerve.setDefaultCommand(new DriveStickSlew(swerve, DRIVER_CONTROLLER));
        shooter.setDefaultCommand(new ShooterOff(shooter));
        climber.setDefaultCommand(new RunClimber(climber, INGESTOR, OPERATOR_CONTROLLER));
        //swerve.setDefaultCommand(new TurtleMode(swerve, driverController));

        JoystickButton selectButton = new JoystickButton(OPERATOR_CONTROLLER, 7); 
        selectButton.whileActiveContinuous(new ManualShoot(shooter, INGESTOR));

        JoystickButton startButton = new JoystickButton(OPERATOR_CONTROLLER, 8); 
        startButton.whileActiveContinuous(new SafeZoneShoot(shooter, INGESTOR, false));

        JoystickButton rightBumper = new JoystickButton(OPERATOR_CONTROLLER, 6);
        rightBumper.whileActiveContinuous(new AutoShoot(swerve, shooter, INGESTOR, OPERATOR_CONTROLLER, DRIVER_CONTROLLER));

        JoystickButton leftBumper = new JoystickButton(OPERATOR_CONTROLLER, 5);
        leftBumper.whileActiveContinuous(new SafeZoneShoot(shooter, INGESTOR, true));

        JoystickButton aButton = new JoystickButton(DRIVER_CONTROLLER, 1);
        aButton.whileActiveContinuous(new UpperHubShoot(shooter, INGESTOR));

        JoystickButton bButton = new JoystickButton(DRIVER_CONTROLLER, 2);
        bButton.whileActiveContinuous(new LowerHubShoot(shooter, INGESTOR));

        // this.setNetworkTablesFlushEnabled(true); //turn off 20ms Dashboard update
        // rate
        LiveWindow.setEnabled(false);
        // add commands to the dashboard so we can run them seperately

        SmartDashboard.putData("Stick Drive", new DriveStick(swerve, DRIVER_CONTROLLER));
        SmartDashboard.putData("Drive Forward 0.5mps", new AutoDrive(swerve, 0.5, 0));
        SmartDashboard.putData("Drive FR 0.5mps", new AutoDrive(swerve, 0.5, 0.5));
        SmartDashboard.putData("Reset Orientation", new ResetOrientation(swerve));

        /*
         * m_chooser.setDefaultOption("Auton1", auton1);
         * m_chooser.addOption("Auton2", auton2);
         * m_chooser.addOption("Auton3", auton3);
         * m_chooser.addOption("Auton4", auton4);
         * m_chooser.addOption("Auton5", auton5);
         * SmartDashboard.putData(m_chooser);
         */

        SmartDashboard.putNumber("Shooting delay", 0.0);
    }

    @Override
    public void disabledInit() {
        ranAuton = false;
        swerve.resetRobot();
        DRIVER_CONTROLLER.setRumble(RumbleType.kLeftRumble, 0.0);
        DRIVER_CONTROLLER.setRumble(RumbleType.kRightRumble, 0.0);
        Shooter.setCoast(false);
        swerve.setBrakeMode(false);
    }

    @Override
    public void autonomousInit() {
       // AutonTwoBall.resetAutonShoot();
        CommandScheduler.getInstance().cancelAll();
        m_selectedAuton = CHOOSER.getSelected();
        //System.out.println("Auton Selected: " + m_selectedAuton);

        // CommandScheduler.getInstance().schedule(new HomeHood(shooter));
        CommandScheduler.getInstance().schedule(new AutonTwoBall(swerve, shooter, INGESTOR));
        //Pose2d pos = swerve.odometry.getPoseMeters();
        ranAuton = true;
    }

    @Override
    public void autonomousPeriodic() {
        // CommandScheduler.getInstance().schedule(new AutonOption0(swerve));
        // Pose2d pos = swerve.odometry.getPoseMeters();
        // swerve.setPosition(10.85, 6.13, 0, 2);
        // swerve.setPosition(8.57, 7.53, 0, 1);

    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        AutonTwoBall.resetAutonShoot();
        if (!ranAuton) {
            // CommandScheduler.getInstance().schedule(new HomeHood(shooter));
        }
    }

    @Override
    public void teleopPeriodic() {
        INGESTOR.runIngestor();
        swerve.runTurtleMode(DRIVER_CONTROLLER);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // have the field position constantly update
        swerve.updateOdometry();
        // SmartDashboard.putNumber("XPosition", odometry.getXPosition());
        // SmartDashboard.putNumber("YPosition", odometry.getYPosition());

        PI.processCargo();
        //pi.processTargets();
        // automatically turn on/off recording
        if (lastEnabled != isEnabled()) {
            // we know the enabled status changed
            if (lastEnabled) {
                // robot stopped, stop recording
                Shuffleboard.stopRecording();
            } else {
                // robot started, start recording
                Shuffleboard.startRecording();
            }
        }
        // save the result for next loop
        lastEnabled = isEnabled();

        SmartDashboard.putData(swerve);
    }
}
