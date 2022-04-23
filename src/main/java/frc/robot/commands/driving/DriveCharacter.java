package frc.robot.commands.driving;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.Collection;

public class DriveCharacter extends CommandBase {
    private final Drivetrain drive;
    private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    private final NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
    private final double[] numberArray = new double[10];
    private final Collection<Double> entries = new ArrayList<>();
    private final TimedRobot robot;
    //private int counter = 0;
    //private double startTime = 0;
    private double priorAutospeed;
    private String data = "";
    private boolean isRunning;

    public DriveCharacter(TimedRobot robot, Drivetrain drive) {
        super();
        this.drive = drive;
        this.robot = robot;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        //startTime = Timer.getFPGATimestamp();
        //counter = 0;
        isRunning = true;
        robot.addPeriodic(this::runCommand, 0.005);
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
        //double elapsedTime = Timer.getFPGATimestamp() - startTime;
        //System.out.println("Robot disabled");
        SwerveModule[] modules = drive.getModules();
        modules[Drivetrain.FL].setDrive(0.0);
        modules[Drivetrain.RL].setDrive(0.0);
        modules[Drivetrain.FR].setDrive(0.0);
        modules[Drivetrain.RR].setDrive(0.0);

        // data processing step
        data = entries.toString();
        int length = data.length();
        data = data.substring(1, length - 1) + ", ";
        telemetryEntry.setString(data);
        entries.clear();
        //System.out.println("Robot disabled");
        //System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
        data = "";
    }

    private void runCommand() {
        if (!isRunning)
            return;
        SwerveModule[] modules = drive.getModules();

        // Retrieve values to send back before telling the motors to do something
        double now = Timer.getFPGATimestamp();

        double leftPosition = modules[Drivetrain.FL].getDistance();
        double leftRate = modules[Drivetrain.FL].getVelocity();

        double rightPosition = modules[Drivetrain.FR].getDistance();
        double rightRate = modules[Drivetrain.FR].getVelocity();

        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(priorAutospeed);

        double leftMotorVolts = motorVolts;
        double rightMotorVolts = motorVolts;

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;

        // command motors to do things
        boolean rotateEntryBoolean = rotateEntry.getBoolean(false);
        modules[Drivetrain.FL].setDrive((rotateEntryBoolean ? -1 : 1) * autospeed);
        modules[Drivetrain.RL].setDrive((rotateEntryBoolean ? -1 : 1) * autospeed);
        modules[Drivetrain.FR].setDrive(autospeed);
        modules[Drivetrain.RR].setDrive(autospeed);
        // drive.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed,
        // autospeed, false);

        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = leftMotorVolts;
        numberArray[4] = rightMotorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        double driveAngle = drive.getAngle();
        numberArray[9] = Math.toRadians(driveAngle);

        // Add data to a string that is uploaded to NT
        for (double num : numberArray) {
            entries.add(num);
        }
        //counter++;
    }
}
