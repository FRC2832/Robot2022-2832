// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunClimber;
import frc.robot.commands.auton.CenterSearchAuton;
import frc.robot.commands.auton.ThreeBallAuton;
import frc.robot.commands.auton.TwoBallAuton;
import frc.robot.commands.driving.CenterToCargo;
import frc.robot.commands.driving.CenterToHub;
import frc.robot.commands.driving.DriveStickSlew;
import frc.robot.commands.shooting.*;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
    private static boolean isAutoShootFinished;
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(2);
    private final Pi pi = new Pi();
    private final Lidar lidar = new Lidar();
    private ColorSensor colorSensor;
    private Drivetrain swerve;
    private Ingestor ingestor;
    private Shooter shooter;
    private Climber climber;
    private Command autonTwoBall;
    private Command autonCenterSearch;
    private Command autonThreeBall;
    //private boolean ranAuton = false;
    private final SendableChooser<Command> autonChooser = new SendableChooser<>();
    //private boolean lastEnabled;
    // Odometry odometry;

    //private String m_autoSelected;
    //private final SendableChooser<String> CHOOSER = new SendableChooser<>();

    public static Command shotCaller(Drivetrain drive, Shooter shooter, Ingestor ingestor,
                                     XboxController driverController, XboxController operatorController) {
        /*if (Pi.getIsTargetCamConnected() && Pi.getIsCargoCamConnected()) {
        }*/ //TODO: Add logic to pick between auto shot and hybrid
        return new AutoShoot(drive, shooter, ingestor, operatorController, driverController);
    }

    public static boolean getIsAutoShootFinished() {
        return isAutoShootFinished;
    }

    public static void setIsAutoShootFinished(boolean finished) {
        isAutoShootFinished = finished;
    }

    @Override
    public void robotInit() {
        colorSensor = new ColorSensor();
        swerve = new Drivetrain(driverController);
        ingestor = new Ingestor(colorSensor, operatorController, driverController);
        Configuration.SetPersistentKeys();
        GitVersion vers = GitVersion.loadVersion();
        vers.printVersions();
        DataLogManager.start();
        // Snapshot.start("http://10.28.32.22:1181/stream.mjpg");

        ShooterConstants.LoadConstants();
        CommandScheduler.getInstance().registerSubsystem(pi, lidar);
        shooter = new Shooter(driverController, operatorController, ingestor);

        climber = new Climber();

        //turtleMode = new TurtleMode(swerve, driverController);

        CommandScheduler.getInstance().registerSubsystem(swerve, colorSensor);
        swerve.setDefaultCommand(new DriveStickSlew(swerve, driverController));
        shooter.setDefaultCommand(new ShooterOff(shooter));
        climber.setDefaultCommand(new RunClimber(climber, operatorController, shooter));
        //swerve.setDefaultCommand(new TurtleMode(swerve, driverController));

        JoystickButton selectButton = new JoystickButton(operatorController, 7);
        selectButton.whileActiveContinuous(new SideShoot(shooter, ingestor));
        JoystickButton startButton = new JoystickButton(operatorController, 8);
        startButton.whileActiveContinuous(new SafeZoneShoot(shooter, ingestor, false));

        JoystickButton rightBumper = new JoystickButton(operatorController, 6);
        rightBumper.whileActiveContinuous(
                new AutoShoot(swerve, shooter, ingestor, operatorController, driverController));

        JoystickButton leftBumper = new JoystickButton(operatorController, 5);
        leftBumper.whileActiveContinuous(new SafeZoneShoot(shooter, ingestor, true));

        JoystickButton aButton = new JoystickButton(driverController, 1);
        aButton.whileActiveContinuous(new HubShoot(shooter, ingestor, true));

        JoystickButton bButton = new JoystickButton(driverController, 2);
        bButton.whileActiveContinuous(new HubShoot(shooter, ingestor, false));

        JoystickButton driverStartButton = new JoystickButton(driverController, 8);
        driverStartButton.whileActiveContinuous(new ShooterBackwards(shooter, ingestor));

        JoystickButton xButton = new JoystickButton(driverController, 3);
        xButton.whileActiveContinuous(new CenterToHub(swerve));

        JoystickButton yButton = new JoystickButton(driverController, 4);
        yButton.whileActiveContinuous(new CenterToCargo(swerve));

        // this.setNetworkTablesFlushEnabled(true); //turn off 20ms Dashboard update
        // rate
        LiveWindow.setEnabled(false);
        // add commands to the dashboard so we can run them seperately

        //SmartDashboard.putData("Stick Drive", new DriveStick(swerve, DRIVER_CONTROLLER));
        //SmartDashboard.putData("Drive Forward 0.5mps", new AutoDrive(swerve, 0.5, 0.0));
        //SmartDashboard.putData("Drive FR 0.5mps", new AutoDrive(swerve, 0.5, 0.5));
        //SmartDashboard.putData("Reset Orientation", new ResetOrientation(swerve));
        SmartDashboard.putBoolean("Two/Three Ball Auton", false);
        SmartDashboard.putBoolean("Skip Reverse Auton Drive", false);
        SmartDashboard.putNumber("Angle Difference", 0.0);
        SmartDashboard.putBoolean("Using Lidar", true);
        SmartDashboard.putNumber("Lidar distance", 0.0);
        SmartDashboard.putNumber("Vision distance", 0.0);
        SmartDashboard.putBoolean("Force use lidar", false);
        SmartDashboard.putBoolean("Two ball (true) / center search (false) Auton", true);

        autonTwoBall = new TwoBallAuton(swerve, shooter, ingestor);
        autonCenterSearch = new CenterSearchAuton(swerve, shooter, ingestor);
        autonThreeBall = new ThreeBallAuton(swerve, shooter, ingestor);

        autonChooser.setDefaultOption("Two ball (anywhere)", autonTwoBall);
        autonChooser.addOption("Search (center, faces hub)", autonCenterSearch);
        autonChooser.addOption("Three ball (right side)", autonThreeBall);
        SmartDashboard.putData(autonChooser);

        //SmartDashboard.putNumber("Shooting delay", 0.0);
    }

    @Override
    public void disabledInit() {
        //ranAuton = false;
        swerve.resetRobot();
        stopControllerRumble(driverController);
        stopControllerRumble(operatorController);
        Shooter.setCoast(false);
        swerve.setBrakeMode(false);
        //climber.setUnlocked(false);
        // Shuffleboard.stopRecording();
        isAutoShootFinished = false;
    }

    @Override
    public void autonomousInit() {
        // TwoBallAuton.resetAutonShoot();
        CommandScheduler.getInstance().cancelAll();

        Command autonCom = autonChooser.getSelected();
        // if (SmartDashboard.getBoolean("Two ball (true) / center search (false) Auton", true)) {
        //     autonCom = new TwoBallAuton(swerve, shooter, ingestor);
        // } else {
        //     autonCom = new CenterSearchAuton(swerve, shooter, ingestor);
        // }

        if (autonCom != null) {
            CommandScheduler.getInstance().schedule(autonCom);
        }
        // Shuffleboard.startRecording();
        //ranAuton = true;
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        TwoBallAuton.resetAutonShoot();
        // Shuffleboard.startRecording();
        /*if (!ranAuton) {
            // CommandScheduler.getInstance().schedule(new HomeHood(shooter));
        }*/
    }

    @Override
    public void testInit() {
        // Shuffleboard.startRecording();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // have the field position constantly update
        //swerve.updateOdometry();
        // SmartDashboard.putNumber("XPosition", odometry.getXPosition());
        // SmartDashboard.putNumber("YPosition", odometry.getYPosition());

        //pi.processCargo();
        //pi.processTargets();
        // automatically turn on/off recording
        /*if (lastEnabled != isEnabled()) {
            // we know the enabled status changed
            if (lastEnabled) {
                Shuffleboard.stopRecording();
                // robot stopped, stop recording
            } else {
                Shuffleboard.startRecording();
                // robot started, start recording
            }
        }*/
        // save the result for next loop
        //lastEnabled = isEnabled();

        //SmartDashboard.putData(swerve);
        //SmartDashboard.putNumber("Shooter Motor Temperature F", shooter.getShooterTemperature());
    }

    @Override
    public void autonomousPeriodic() {
        // CommandScheduler.getInstance().schedule(new Option0Auton(swerve));
        // Pose2d pos = swerve.odometry.getPoseMeters();
        // swerve.setPosition(10.85, 6.13, 0, 2);
        // swerve.setPosition(8.57, 7.53, 0, 1);
        SmartDashboard.putData(swerve);
    }

    @Override
    public void teleopPeriodic() {
        ingestor.runIngestor();
        swerve.runTurtleMode(driverController);
        SmartDashboard.putData(swerve);
    }

    public static void stopControllerRumble(XboxController controller) {
        rumbleController(controller, 0.0);
    }

    public static void rumbleController(XboxController controller, double rumbleSpeed) {
        controller.setRumble(RumbleType.kLeftRumble, rumbleSpeed);
        controller.setRumble(RumbleType.kRightRumble, rumbleSpeed);
    }
}