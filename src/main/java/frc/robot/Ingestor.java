package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ColorSensor.CargoColor;

//import edu.wpi.first.wpilibj.DigitalOutput;

public class Ingestor extends SubsystemBase {
    private static final double TRIGGER_SENSITIVITY = 0.5;
    // Targeted motor speeds
    private static final double INGESTOR_SPEED = 0.9; // 1000.0;
    private static final double STAGE_1_SPEED = 0.75;// 1000.0;
    private static final double STAGE_2_SPEED = 0.85; // 1000.0;
    private final WPI_TalonSRX ingestorWheels;
    // private WPI_TalonSRX ingestorGate;
    private final WPI_TalonSRX stage1Conveyor;
    private final WPI_TalonSRX stage2Conveyor;
    private final CANSparkMax ingestorLift;
    private final RelativeEncoder altEncoder;
    private final SparkMaxPIDController liftPidController;
    // private XboxController driverController;
    private final XboxController operatorController;
    private final XboxController driverController;
    private final Timer sendTimer;
    private final Timer stageTimer;
    private final DigitalInput stage1ProxSensor;
    //private final Counter counter;
    private boolean sendTimerStarted;
    private boolean stageTimerStarted;
    private boolean hasCargo;
    // private SmartDashboard smartDashboard;
    private boolean triggerPressed;
    private static int totalBalls;
    // private ColorSensor stage2ColorSensor;
    // private boolean ballAtColorSensor;
    private double liftRotations;
    //private static final double INGESTOR_LIFT_SPEED = 0.25;

    public Ingestor(ColorSensor colorSensor, XboxController operatorController, XboxController driverController) {
        ingestorWheels = new WPI_TalonSRX(CanIDConstants.INTAKE_WHEELS);
        // ingestorGate = new WPI_TalonSRX(2);
        stage1Conveyor = new WPI_TalonSRX(CanIDConstants.STAGE_1);
        stage2Conveyor = new WPI_TalonSRX(CanIDConstants.STAGE_2);
        CANSparkMax.MotorType BRUSHLESS = CANSparkMax.MotorType.valueOf("kBrushless");
        ingestorLift = new CANSparkMax(CanIDConstants.INTAKE_LIFT, BRUSHLESS);
        altEncoder = ingestorLift.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        // driverController = new XboxController(0);
        this.operatorController = operatorController;
        this.driverController = driverController;
        sendTimer = new Timer();
        stageTimer = new Timer();
        // Port port = Port.kOnboard; // TODO: Need to verify this.
        // stage1ProxSensor = new DigitalInput(0);
        stage1ProxSensor = new DigitalInput(0);
        //counter = new Counter(stage1ProxSensor);
        totalBalls = 0;
        // ballAtColorSensor = false;

        liftPidController = ingestorLift.getPIDController();
        liftPidController.setFeedbackDevice(altEncoder);

        // PID coefficients
        double kP = 2.0;
        double kI = 0.0;
        double kD = 0.0;
        double kIz = 0.0;
        double kFF = 0.0;
        double kMaxOutput = 1.0;
        double kMinOutput = -1.0;

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

        SmartDashboard.putNumber("Ingestor motor applied output", ingestorLift.getAppliedOutput());
        SmartDashboard.putNumber("alt encoder velocity", altEncoder.getVelocity());
        SmartDashboard.putNumber("alt encoder position", altEncoder.getPosition());

        // System.out.println("counter - " + counter.get());
        // prox sensor checking
        //if (counter.get() > 0) {
        totalBalls = 0;
        if(getStage1Proximity()){
            // System.out.println("Ball ingested!");
            totalBalls++;
            // System.out.println("Total Balls 1:" + totalBalls);
            /*
            if (counter.get() >= 2) {
                totalBalls = 0;
                counter.reset();
            }
            */
        }
        if (ColorSensor.getCargoColor() != CargoColor.Unknown) {
            totalBalls++;
        }
        if ((totalBalls > 0 && !hasCargo) || (hasCargo && totalBalls == 0)) {
            SmartDashboard.putNumber("Total Balls", totalBalls);
            hasCargo = !hasCargo;
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
        
        if (operatorController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY || driverController.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY) { // ingestor down and in
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
            if (stageTimer.get() < 2.0 && triggerPressed) { // triggerPressed is so stage 1 doesn't run at enabling
                stage1ConveyorSpeed = STAGE_1_SPEED;
            }
        }

        if (operatorController.getXButton()) { // push ball to stage 1
            stage2ConveyorSpeed = STAGE_2_SPEED;
        } else if (operatorController.getYButton()) { // push ball to shooter
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

    public void lowerIngestor() {
        liftRotations = -0.165;
        double position = altEncoder.getPosition();
        if (position > liftRotations + (Math.abs(liftRotations) * 0.02)) {
            liftPidController.setReference(liftRotations, ControlType.kPosition);
        } else {
            ingestorLift.set(0.0); // once it's 98% of the way there let it drop
        }
        ingestorLift.setIdleMode(IdleMode.kCoast);
    }

    public void liftIngestor() {
        liftRotations = 0.0;
        liftPidController.setReference(liftRotations, ControlType.kPosition);
        ingestorLift.setIdleMode(IdleMode.kBrake);
    }

    public boolean sendCargoToShooter() {
        // TODO: replace timer with prox/color sensor
        if (sendTimer.get() < 0.001) {
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
            sendTimer.stop();
            sendTimer.reset();
            sendTimerStarted = false;
            /*totalBalls = 0;
            //Check if a ball is in 1st slip stream
            if(getStage1Proximity()){
                //TODO: Check for ball in second stream and then send ball to second slip stream if empty
                totalBalls++;
            }
            //TODO: Add second proximity sensor detection 
            if (ColorSensor.getCargoColor() != CargoColor.Unknown) {
                totalBalls++;
            }
            SmartDashboard.putNumber("Total Balls", totalBalls);*/
            return true;
        }
        return false;
    }

    public boolean sendOneCargoToShooter() {
        // TODO: replace timer with prox/color sensor
        if (!sendTimerStarted) {
            sendTimer.start(); // TODO: This method's boolean return isn't being used? Might that cause issues?
            sendTimerStarted = true;
        }
        if (sendTimer.get() < 3.0) {
            stage2Conveyor.set(-STAGE_2_SPEED);
        } else {
            sendTimer.stop();
            sendTimer.reset();
            sendTimerStarted = false;
            return true;
        }
        return false;
    }

    public void runStage1Out() {
        stage1Conveyor.set(-STAGE_1_SPEED);
    }

    public void runStage2Out() {
        stage2Conveyor.set(STAGE_2_SPEED);
    }

    public void runStage1In() {
        stage1Conveyor.set(STAGE_1_SPEED);
    }

    public void runStage2In() {
        stage2Conveyor.set(STAGE_2_SPEED);
    }

    public void stopStage1() {
        stage1Conveyor.set(0.0);
    }

    public void stopStage2() {
        stage2Conveyor.set(0.0);
    }

    public void runIngestorOut() {
        ingestorWheels.set(INGESTOR_SPEED);
    }

    public void stopIngestorWheels() {
        ingestorWheels.set(0);
    }

    public void threeBallAutonIngest() {
        stage1Conveyor.set(ControlMode.PercentOutput, STAGE_1_SPEED);
        ingestorWheels.set(ControlMode.PercentOutput, -INGESTOR_SPEED);
    }

    public boolean getStage1Proximity() {
        return !stage1ProxSensor.get();
    }

    /*public int getStage2Proximity() {
        return ColorSensor.getProx();
    } */

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