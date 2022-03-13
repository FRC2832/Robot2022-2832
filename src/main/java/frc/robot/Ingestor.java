package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;

public class Ingestor extends SubsystemBase{
    private WPI_TalonSRX ingestorWheels;
    private ColorSensorV3 stage2ColorSensor;
    //private WPI_TalonSRX ingestorGate;
    private WPI_TalonSRX stage1Conveyor;
    private WPI_TalonSRX stage2Conveyor;
    private CANSparkMax ingestorLift;
    private final CANSparkMax.MotorType BRUSHLESS = CANSparkMax.MotorType.valueOf("kBrushless");
    // private XboxController driverController;
    private XboxController operatorController;
    private static final double TRIGGER_SENSITIVITY = 0.5;
    private Timer timer;
    private boolean timerStarted = false;
    private DigitalInput stage1ProxSensor;

    // Targetted motor speeds
    private static final double INGESTOR_SPEED = 0.75; //1000.0;
    private static final double STAGE_1_SPEED = 0.75;//1000.0;
    private static final double STAGE_2_SPEED = 0.85; //1000.0;
    private static final double INGESTOR_LIFT_SPEED = 0.25;

    public Ingestor() {
        ingestorWheels = new WPI_TalonSRX(27);
        // ingestorGate = new WPI_TalonSRX(2);
        stage1Conveyor = new WPI_TalonSRX(25);
        stage2Conveyor = new WPI_TalonSRX(23);
        ingestorLift = new CANSparkMax(31, BRUSHLESS);
        // driverController = new XboxController(0);
        operatorController = new XboxController(2);
        timer = new Timer();
        Port port = Port.kOnboard; // TODO: Need to verify this.
        stage2ColorSensor = new ColorSensorV3(port);
        //stage1ProxSensor = new DigitalInput(0);
    }

    public void runIngestor() {
        // color sensor conditions
        if(stage2ColorSensor.getProximity() > 1000){
            System.out.println("getProximity() > 1000");
        }
        
        Color sensorColor = stage2ColorSensor.getColor();
        if (stage2ColorSensor.getBlue() > 128) {
            // System.out.println("getBlue() returned more than 128");
        }
        if (stage2ColorSensor.getRed() > 128) {
            // System.out.println("getRed() returned more than 128");
        }
        if (sensorColor.blue > 128.0) {
            System.out.println("sensorColor.blue is more than 128.0");
        }
        if (sensorColor.red > 128.0) {
            System.out.println("sensorColor.red is more than 128.0");
        }
      
        if(operatorController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY){ //ingestor in
            ingestorWheels.set(-INGESTOR_SPEED);
            stage1Conveyor.set(STAGE_1_SPEED);
        } else if(operatorController.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY){ //ingestor out
            ingestorWheels.set(INGESTOR_SPEED);
            stage1Conveyor.set(-STAGE_1_SPEED);
        } else {
            ingestorWheels.set(0);
            stage1Conveyor.set(0);
        }

        if (operatorController.getXButton()) { // push ball to shooter
            stage2Conveyor.set(STAGE_2_SPEED);
        } else if (operatorController.getYButton()) { // push ball to stage 1
            stage2Conveyor.set(-STAGE_2_SPEED);
        } else {
            stage2Conveyor.set(0.0);
        }

        if (operatorController.getBButton()) { // lift ingestor
            ingestorLift.set(INGESTOR_LIFT_SPEED);
        } else if (operatorController.getAButton()) { // lower ingestor
            ingestorLift.set(-INGESTOR_LIFT_SPEED);
        } else {
            ingestorLift.set(0.0);
        }
    }

    public void sendCargoToShooter() {
        // TODO: replace timer with prox/color sensor
        if (!timerStarted) {
            timer.start();
            timerStarted = true;
        }
        if (timer.get() < 1) {
            stage2Conveyor.set(-STAGE_2_SPEED);
        } else if(timer.get() < 5) {
            stage2Conveyor.set(-STAGE_2_SPEED);
            stage1Conveyor.set(STAGE_1_SPEED);
        } else {
            timer.reset();
            timerStarted = false;
        }
    }

    public void liftIngestor() {
        ingestorLift.set(INGESTOR_LIFT_SPEED);
        // TODO: stop when ingestor is all the way up
    }


    public boolean getStage1Proximity(){
        return stage1ProxSensor.get();
    } 

    public WPI_TalonSRX getIngestorWheels(){
        return ingestorWheels;
    }

    public WPI_TalonSRX getStage1Conveyor(){
        return stage1Conveyor;
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