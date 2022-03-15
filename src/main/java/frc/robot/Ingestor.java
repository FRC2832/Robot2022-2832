package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;

public class Ingestor extends SubsystemBase {
    private WPI_TalonSRX ingestorWheels;
    private ColorSensorV3 stage2ColorSensor;
    //private WPI_TalonSRX ingestorGate;
    private WPI_TalonSRX stage1Conveyor;
    private WPI_TalonSRX stage2Conveyor;
    private CANSparkMax ingestorLift;
    private final CANSparkMax.MotorType BRUSHLESS = CANSparkMax.MotorType.valueOf("kBrushless");
    private Timer timer;
    private boolean timerStarted = false;

    // Targetted motor speeds
    private static final double INGESTOR_SPEED = 0.75; 
    private static final double STAGE_1_SPEED = 0.75;
    private static final double STAGE_2_SPEED = 0.85; 
    private static final double INGESTOR_LIFT_SPEED = 0.25;

    public Ingestor() {
        ingestorWheels = new WPI_TalonSRX(27);
        // ingestorGate = new WPI_TalonSRX(2);
        stage1Conveyor = new WPI_TalonSRX(25);
        stage2Conveyor = new WPI_TalonSRX(23);
        ingestorLift = new CANSparkMax(31, BRUSHLESS);
        timer = new Timer();
        Port port = Port.kOnboard; // TODO: Need to verify this.
        stage2ColorSensor = new ColorSensorV3(port);
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

    public void lowerIngestor() {
        ingestorLift.set(-INGESTOR_LIFT_SPEED);
    }

    public void holdIngestor() {
        ingestorLift.set(0.0);
    }

    public void intake() {
        ingestorWheels.set(-INGESTOR_SPEED);
        stage1Conveyor.set(STAGE_1_SPEED);
    }

    public void outtake() {
        ingestorWheels.set(INGESTOR_SPEED);
        stage1Conveyor.set(-STAGE_1_SPEED);
    }

    public void holdWheels() {
        ingestorWheels.set(0);
        stage1Conveyor.set(0);
    }

    public void stage2Forward() {
        stage2Conveyor.set(STAGE_2_SPEED);
    }

    public void stage2Back() {
        stage2Conveyor.set(-STAGE_2_SPEED);
    }

    public void stage2Hold() {
        stage2Conveyor.set(0.0);
    }

}