package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;

public class DriveCharacter extends CommandBase {
    private Drivetrain drive;
    private TimedRobot robot;

    NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
    double[] numberArray = new double[10];
    ArrayList<Double> entries = new ArrayList<Double>();
    int counter = 0;
    double startTime = 0;
    double priorAutospeed = 0;
    String data = "";
    boolean running = false;
    
    public DriveCharacter(TimedRobot robot, Drivetrain drive) {
        this.drive = drive;
        this.robot = robot;
        addRequirements(drive);
    }

    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        counter = 0;
        running = true;
        robot.addPeriodic(this::runCommand, 0.005);
    }

    public void end(boolean interrupted) {
        running = false;
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        System.out.println("Robot disabled");
        var modules = drive.getModules();
        modules[Drivetrain.FL].setDrive(0);
        modules[Drivetrain.RL].setDrive(0);
        modules[Drivetrain.FR].setDrive(0);
        modules[Drivetrain.RR].setDrive(0);

        // data processing step
        data = entries.toString();
        data = data.substring(1, data.length() - 1) + ", ";
        telemetryEntry.setString(data);
        entries.clear();
        System.out.println("Robot disabled");
        System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
        data = "";
    }

    public void runCommand() {
        if(!running) return;
        var modules = drive.getModules();

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
        modules[Drivetrain.FL].setDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed);
        modules[Drivetrain.RL].setDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed);
        modules[Drivetrain.FR].setDrive(autospeed);
        modules[Drivetrain.RR].setDrive(autospeed);
        //drive.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed, false);

        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = leftMotorVolts;
        numberArray[4] = rightMotorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = Math.toRadians(drive.getAngle());

        // Add data to a string that is uploaded to NT
        for (double num : numberArray) {
            entries.add(num);
        }
        counter++;
    }
}
