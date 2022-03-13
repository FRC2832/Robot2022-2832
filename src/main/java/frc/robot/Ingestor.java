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
    private static final double INGESTOR_SPEED = 0.38; //1000.0;
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
        if(operatorController.getAButton()){
            ingestorWheels.set(INGESTOR_SPEED);
            stage1Conveyor.set(-STAGE_1_SPEED);
        } else if(operatorController.getBButton()){
            ingestorWheels.set(-INGESTOR_SPEED);
            stage1Conveyor.set(STAGE_1_SPEED);
        } else {
            ingestorWheels.set(0);
            stage1Conveyor.set(0);
        }

        // if (operatorController.getLeftBumper()) { // ingest ball
        //     //ingestorWheels.set(ControlMode.Velocity, INGESTOR_SPEED);
        //     ingestorWheels.set(INGESTOR_SPEED);
        // } else if (operatorController.getRightBumper()) { // expell ball
        //     //ingestorWheels.set(ControlMode.Velocity, -INGESTOR_SPEED);
        //     ingestorWheels.set(-INGESTOR_SPEED);
        // } else {
        //     //ingestorWheels.set(ControlMode.Velocity, 0.0);
        //     ingestorWheels.set(0.0);
        // }
        // if (operatorController.getAButton()) { // push ball to stage 2
        //     stage1Conveyor.set(STAGE_1_SPEED);
        //     //stage1Conveyor.set(ControlMode.Velocity, STAGE_1_SPEED);
        // } else if (operatorController.getBButton()) { // push ball to ingestor
        //     stage1Conveyor.set(-STAGE_1_SPEED);
        //     //stage1Conveyor.set(ControlMode.Velocity, -STAGE_1_SPEED);
        // } else {
        //     //stage1Conveyor.set(ControlMode.Velocity, 0.0);
        //     stage1Conveyor.set(0.0);
        // }
        if (operatorController.getXButton()) { // push ball to shooter
            stage2Conveyor.set(STAGE_2_SPEED);
            //stage2Conveyor.set(ControlMode.Velocity, STAGE_2_SPEED);
        } else if (operatorController.getYButton()) { // push ball to stage 1
            stage2Conveyor.set(-STAGE_2_SPEED);
            //stage2Conveyor.set(ControlMode.Velocity, -STAGE_2_SPEED);
        } else {
            //stage2Conveyor.set(ControlMode.Velocity, 0.0);
            stage2Conveyor.set(0.0);
        }
        if (operatorController.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY) { // lift ingestor
            ingestorLift.set(INGESTOR_LIFT_SPEED);
        } else if (operatorController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY) { // lower ingestor
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
        if (timer.get() < 5) {
            stage2Conveyor.set(-STAGE_2_SPEED);
        } else {
            timer.reset();
            timerStarted = false;
        }
    }

    public void sendOneCargoToShooter() {
        // TODO: replace timer with prox/color sensor
        if (!timerStarted) {
            timer.start();
            timerStarted = true;
        }
        if (timer.get() < 5) {
            stage2Conveyor.set(-STAGE_2_SPEED);
        } else {
            timer.reset();
            timerStarted = false;
        }
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