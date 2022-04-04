package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import edu.wpi.first.wpilibj.DigitalOutput;

public class Ingestor extends SubsystemBase {
    private WPI_TalonSRX ingestorWheels;
    // private WPI_TalonSRX ingestorGate;
    private WPI_TalonSRX stage1Conveyor;
    private WPI_TalonSRX stage2Conveyor;
    private CANSparkMax ingestorLift;
    private RelativeEncoder altEncoder;
    private SparkMaxPIDController liftPidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private final CANSparkMax.MotorType BRUSHLESS = CANSparkMax.MotorType.valueOf("kBrushless");
    // private XboxController driverController;
    private XboxController operatorController;
    private static final double TRIGGER_SENSITIVITY = 0.5;
    private Timer sendTimer;
    private Timer stageTimer;
    private boolean sendTimerStarted = false;
    private boolean stageTimerStarted = false;
    private boolean triggerPressed = false;
    private DigitalInput stage1ProxSensor;
    private Counter counter;
    private int totalBalls;
    // private ColorSensor stage2ColorSensor;
    // private boolean ballAtColorSensor;
    private double liftRotations;
    // private SmartDashboard smartDashboard;

    // Targetted motor speeds
    private static final double INGESTOR_SPEED = 0.75; // 1000.0;
    private static final double STAGE_1_SPEED = 0.75;// 1000.0;
    private static final double STAGE_2_SPEED = 0.85; // 1000.0;
    private static final double INGESTOR_LIFT_SPEED = 0.25;

    public Ingestor(ColorSensor colorSensor) {
        ingestorWheels = new WPI_TalonSRX(CanIDConstants.INTAKE_WHEELS);
        // ingestorGate = new WPI_TalonSRX(2);
        stage1Conveyor = new WPI_TalonSRX(CanIDConstants.STAGE_1);
        stage2Conveyor = new WPI_TalonSRX(CanIDConstants.STAGE_2);
        ingestorLift = new CANSparkMax(CanIDConstants.INTAKE_LIFT, BRUSHLESS);
        altEncoder = ingestorLift.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        // driverController = new XboxController(0);
        operatorController = new XboxController(2);
        sendTimer = new Timer();
        stageTimer = new Timer();
        // Port port = Port.kOnboard; // TODO: Need to verify this.
        // stage1ProxSensor = new DigitalInput(0);
        stage1ProxSensor = new DigitalInput(0);
        counter = new Counter(stage1ProxSensor);
        totalBalls = 0;
        // ballAtColorSensor = false;

        liftPidController = ingestorLift.getPIDController();
        liftPidController.setFeedbackDevice(altEncoder);

        // PID coefficients
        kP = 2.5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        liftPidController.setP(kP);
        liftPidController.setI(kI);
        liftPidController.setD(kD);
        liftPidController.setIZone(kIz);
        liftPidController.setFF(kFF);
        liftPidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        // SmartDashboard.putNumber("P Gain", kP);
        // SmartDashboard.putNumber("I Gain", kI);
        // SmartDashboard.putNumber("D Gain", kD);
        // SmartDashboard.putNumber("I Zone", kIz);
        // SmartDashboard.putNumber("Feed Forward", kFF);
        // SmartDashboard.putNumber("Max Output", kMaxOutput);
        // SmartDashboard.putNumber("Min Output", kMinOutput);
        // SmartDashboard.putNumber("Set Rotations", 0);

    }


    public void runIngestor() {

        // SmartDashboard.putNumber("Ingestor motor applied output",
        // ingestorLift.getAppliedOutput());
        // SmartDashboard.putNumber("alt encoder velocity", altEncoder.getVelocity());
        // SmartDashboard.putNumber("alt encoder position", altEncoder.getPosition());

        // System.out.println("counter - " + counter.get());
        // prox sensor checking
        if (counter.get() > 0) {
            // System.out.println("Ball ingested!");
            totalBalls++;
            // System.out.println("Total Balls 1:" + totalBalls);
            SmartDashboard.putNumber("Total Balls", totalBalls);
            if (counter.get() >= 2) {
                totalBalls = 0;
                counter.reset();
            }
        }

        // color sensor conditions
        /*
         * if (ColorSensor.getProx() > 1000) {
         * //System.out.println("getProximity() > 1000");
         * //ballAtColorSensor = true;
         * //System.out.println("Total Balls 2:" + totalBalls);
         * SmartDashboard.putNumber("Total Balls", totalBalls);
         * } else {
         * //System.out.println("getProximity() <= 1000");
         * //ballAtColorSensor = false;
         * //System.out.println("Total Balls 3:" + totalBalls);
         * SmartDashboard.putNumber("Total Balls", totalBalls);
         * }
         */
        /*
         * if(!stage1ProxSensor.get() && !ballAtColorSensor){
         * totalBalls--;
         * System.out.println("Total Balls " + totalBalls);
         * SmartDashboard.putNumber("Total Balls", totalBalls);
         * }
         */

        // String cargoColor = ColorSensor.getCargoColor();
        /*
         * if (cargoColor == CargoColor.Blue) {
         * System.out.println("Cargo is blue");
         * } else if (cargoColor == CargoColor.Red) {
         * System.out.println("Cargo is red");
         * }
         */
        double ingestorWheelSpeed = 0.0;
        double stage1ConveyorSpeed = 0.0;
        double stage2ConveyorSpeed = 0.0;
        // double ingestorLiftSpeed = 0.0;

        if (operatorController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY) { // ingestor down and in
            ingestorWheelSpeed = -INGESTOR_SPEED;
            stage1ConveyorSpeed = STAGE_1_SPEED;
            lowerIngestor();
            stageTimer.reset();
            triggerPressed = true;
        } else if (operatorController.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY) { // ingestor down and out
            ingestorWheelSpeed = INGESTOR_SPEED;
            stage1ConveyorSpeed = -STAGE_1_SPEED;
            lowerIngestor();
            stageTimer.reset();
            triggerPressed = true;
        } else { // ingestor up, run stage 1 for two more seconds
            liftIngestor();
            if (!stageTimerStarted) {
                stageTimer.start();
                stageTimerStarted = true;
            }
            if (stageTimer.get() < 2 && triggerPressed) { // triggerPressed is so stage 1 doesn't run at enabling
                stage1ConveyorSpeed = STAGE_1_SPEED;
            } else {
                stage1ConveyorSpeed = 0.0;
            }
        }

        if (operatorController.getXButton()) { // push ball to shooter
            stage2ConveyorSpeed = STAGE_2_SPEED;
        } else if (operatorController.getYButton()) { // push ball to stage 1
            stage2ConveyorSpeed = -STAGE_2_SPEED;
        }

        // if (operatorController.getBButton()) { // lift ingestor
        // // ingestorLiftSpeed = INGESTOR_LIFT_SPEED;
        // rotations = 0;
        // } else if (operatorController.getAButton()) { // lower ingestor
        // // ingestorLiftSpeed = -INGESTOR_LIFT_SPEED;
        // rotations = -0.15;
        // }

        ingestorWheels.set(ingestorWheelSpeed);
        stage1Conveyor.set(stage1ConveyorSpeed);
        stage2Conveyor.set(stage2ConveyorSpeed);
        // ingestorLift.set(ingestorLiftSpeed);
        // liftPidController.setReference(liftRotations, ControlType.kPosition);

        // SmartDashboard.putNumber("SetPoint", liftRotations);
        // SmartDashboard.putNumber("ProcessVariable", altEncoder.getPosition());
    }

    public boolean sendCargoToShooter() {
        // TODO: replace timer with prox/color sensor
        if (!sendTimerStarted) {
            sendTimer.start();
            sendTimerStarted = true;
        }
        double sendTimerVal = sendTimer.get();
        if (sendTimerVal < 0.5) {
            stage2Conveyor.set(-STAGE_2_SPEED);
        } else if (sendTimerVal < 2.5) {
            stage2Conveyor.set(-STAGE_2_SPEED);
            stage1Conveyor.set(STAGE_1_SPEED);
        } else {
            sendTimer.reset();
            sendTimerStarted = false;
            return true;
        }
        return false;
    }

    public boolean sendOneCargoToShooter() {
        // TODO: replace timer with prox/color sensor
        if (!sendTimerStarted) {
            sendTimer.start();
            sendTimerStarted = true;
        }
        if (sendTimer.get() < 3) {
            stage2Conveyor.set(-STAGE_2_SPEED);
        } else {
            sendTimer.reset();
            sendTimerStarted = false;
            return true;
        }
        return false;
    }

    public void liftIngestor() {
        liftRotations = 0.0;
        liftPidController.setReference(liftRotations, ControlType.kPosition);
    }

    public void lowerIngestor() {
        liftRotations = -0.165;
        liftPidController.setReference(liftRotations, ControlType.kPosition);
    }

    public void threeBallAutonIngest() {
        stage1Conveyor.set(ControlMode.PercentOutput, STAGE_1_SPEED);
        ingestorWheels.set(ControlMode.PercentOutput, -INGESTOR_SPEED);
    }

    public boolean getStage1Proximity() {
        return stage1ProxSensor.get();
    }

    public int getStage2Proximity() {
        return ColorSensor.getProx();
    }

    public WPI_TalonSRX getIngestorWheels() {
        return ingestorWheels;
    }

    public WPI_TalonSRX getStage1Conveyor() {
        return stage1Conveyor;
    }

    public WPI_TalonSRX getStage2Conveyor() {
        return stage2Conveyor;
    }

    /*
     * public void ingest(){
     * ingestorWheels.set(ControlMode.Velocity, 1000);
     * }
     * 
     * public void expel(){
     * ingestorWheels.set(ControlMode.Velocity, -1000);
     * }
     * 
     * public void stage1(){
     * 
     * }
     */
}