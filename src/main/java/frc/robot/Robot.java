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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutonThreeBall;
import frc.robot.commands.AutonTwoBall;
import frc.robot.commands.DriveStickSlew;
import frc.robot.commands.HubShoot;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.RunClimber;
import frc.robot.commands.SafeZoneShoot;
import frc.robot.commands.ShooterBackwards;
import frc.robot.commands.ShooterOff;

public class Robot extends TimedRobot {
    private final XboxController DRIVER_CONTROLLER = new XboxController(0);
    private final XboxController OPERATOR_CONTROLLER = new XboxController(2);
    private final Pi PI = new Pi();
    private ColorSensor colorSensor;
    private Drivetrain swerve;
    private Ingestor ingestor;
    private Shooter shooter;
    private Climber climber;
    //private boolean ranAuton = false;

    private boolean lastEnabled;
    // Odometry odometry;

    //private String m_autoSelected;
    //private final SendableChooser<String> CHOOSER = new SendableChooser<>();

    @Override
    public void robotInit() {
        colorSensor = new ColorSensor();
        swerve = new Drivetrain(DRIVER_CONTROLLER);
        ingestor = new Ingestor(colorSensor, OPERATOR_CONTROLLER);
        Configuration.SetPersistentKeys();
        GitVersion vers = GitVersion.loadVersion();
        vers.printVersions();
        DataLogManager.start();
        // Snapshot.start("http://10.28.32.22:1181/stream.mjpg");

        ShooterConstants.LoadConstants();
        CommandScheduler.getInstance().registerSubsystem(PI);
        shooter = new Shooter(DRIVER_CONTROLLER, OPERATOR_CONTROLLER, ingestor);

        climber = new Climber(ingestor);

        //turtleMode = new TurtleMode(swerve, driverController);

        CommandScheduler.getInstance().registerSubsystem(swerve, colorSensor);
        swerve.setDefaultCommand(new DriveStickSlew(swerve, DRIVER_CONTROLLER));
        shooter.setDefaultCommand(new ShooterOff(shooter));
        climber.setDefaultCommand(new RunClimber(climber, OPERATOR_CONTROLLER));
        //swerve.setDefaultCommand(new TurtleMode(swerve, driverController));

        JoystickButton selectButton = new JoystickButton(OPERATOR_CONTROLLER, 7);
        selectButton.whileActiveContinuous(new ManualShoot(shooter, ingestor));

        JoystickButton startButton = new JoystickButton(OPERATOR_CONTROLLER, 8);
        startButton.whileActiveContinuous(new SafeZoneShoot(shooter, ingestor, false));

        JoystickButton rightBumper = new JoystickButton(OPERATOR_CONTROLLER, 6);
        rightBumper.whileActiveContinuous(
                new AutoShoot(swerve, shooter, ingestor, OPERATOR_CONTROLLER, DRIVER_CONTROLLER));

        JoystickButton leftBumper = new JoystickButton(OPERATOR_CONTROLLER, 5);
        leftBumper.whileActiveContinuous(new SafeZoneShoot(shooter, ingestor, true));

        JoystickButton aButton = new JoystickButton(DRIVER_CONTROLLER, 1);
        aButton.whileActiveContinuous(new HubShoot(shooter, ingestor, true));

        JoystickButton bButton = new JoystickButton(DRIVER_CONTROLLER, 2);
        bButton.whileActiveContinuous(new HubShoot(shooter, ingestor, false));

        JoystickButton driverStartButton = new JoystickButton(DRIVER_CONTROLLER, 8);
        driverStartButton.whileActiveContinuous(new ShooterBackwards(shooter, ingestor));

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

        /*
         * m_chooser.setDefaultOption("Auton1", auton1);
         * m_chooser.addOption("Auton2", auton2);
         * m_chooser.addOption("Auton3", auton3);
         * m_chooser.addOption("Auton4", auton4);
         * m_chooser.addOption("Auton5", auton5);
         * SmartDashboard.putData(m_chooser);
         */

        //SmartDashboard.putNumber("Shooting delay", 0.0);
    }

    @Override
    public void disabledInit() {
        //ranAuton = false;
        swerve.resetRobot();
        stopControllerRumble(DRIVER_CONTROLLER);
        stopControllerRumble(OPERATOR_CONTROLLER);
        Shooter.setCoast(false);
        swerve.setBrakeMode(false);
    }

    @Override
    public void autonomousInit() {
        // AutonTwoBall.resetAutonShoot();
        CommandScheduler.getInstance().cancelAll();
        //m_selectedAuton = CHOOSER.getSelected();
        //System.out.println("Auton Selected: " + m_selectedAuton);

        // CommandScheduler.getInstance().schedule(new HomeHood(shooter));
        Command autonCom;
        if (SmartDashboard.getBoolean("Two/Three Ball Auton", false)) {
            autonCom = new AutonTwoBall(swerve, shooter, ingestor);
        } else {
            autonCom = new AutonThreeBall(swerve, shooter, ingestor);
        }

        CommandScheduler.getInstance().schedule(autonCom);
        //ranAuton = true;
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        AutonTwoBall.resetAutonShoot();
        /*if (!ranAuton) {
            // CommandScheduler.getInstance().schedule(new HomeHood(shooter));
        }*/
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
        SmartDashboard.putBoolean("Stage 1 Proximity", ingestor.getStage1Proximity());
        SmartDashboard.putNumber("Shooter Motor Temperature F", shooter.getShooterTemperature());
    }

    @Override
    public void autonomousPeriodic() {
        // CommandScheduler.getInstance().schedule(new AutonOption0(swerve));
        // Pose2d pos = swerve.odometry.getPoseMeters();
        // swerve.setPosition(10.85, 6.13, 0, 2);
        // swerve.setPosition(8.57, 7.53, 0, 1);

    }

    @Override
    public void teleopPeriodic() {
        ingestor.runIngestor();
        swerve.runTurtleMode(DRIVER_CONTROLLER);
    }

    public static void stopControllerRumble(XboxController controller) {
        rumbleController(controller, 0.0);
    }

    public static void rumbleController(XboxController controller, double rumbleSpeed) {
        controller.setRumble(RumbleType.kLeftRumble, rumbleSpeed);
        controller.setRumble(RumbleType.kRightRumble, rumbleSpeed);
    }
}
