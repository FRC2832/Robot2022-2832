package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private WPI_TalonFX rung1;
    private WPI_TalonFX rung23;
    private static final double CLIMBER_UP_SPEED = -0.6;
    private static final double CLIMBER_DOWN_SPEED = 0.4;
    private Ingestor ingestor;
    // private ColorSensor colorSensor;

    public Climber(Ingestor ingestor) {
        rung1 = new WPI_TalonFX(CanIDConstants.RUNG_1_2_WINCH);
        rung23 = new WPI_TalonFX(CanIDConstants.RUNG_3_4_WINCH);
        rung1.setNeutralMode(NeutralMode.Brake);
        rung23.setNeutralMode(NeutralMode.Brake);
        // colorSensor = new ColorSensor();
        // ingestor = new Ingestor(colorSensor);
        this.ingestor = ingestor;
    }

    public void arm1Up() {
        rung1.set(CLIMBER_UP_SPEED);
    }

    public void arm1Down() {
        rung1.set(CLIMBER_DOWN_SPEED);
    }

    public void arm1Hold() {
        rung1.set(0);
    }

    public void arm2Up() {
        ingestor.lowerIngestor();
        rung23.set(-CLIMBER_UP_SPEED);
    }

    public void arm2Down() {
        ingestor.liftIngestor();
        rung23.set(-CLIMBER_DOWN_SPEED);
    }

    public void arm2Hold() {
        rung23.set(0);
    }

    //COMMANDS FOR NEW ROTATING CLIMBER ARM
    public void raiseArm(){ //dpad up
        rung1.set(CLIMBER_UP_SPEED);
    }

    public void lowerArm(){ //dpad down
        rung1.set(CLIMBER_DOWN_SPEED);
    }

    public void rotateArmUp(){ //dpad left
        //run motor
    }

    public void rotateArmDown(){ //dpad right
        //run motor in opposite direction
    }

    public void clampHookOne(){ //a button
        //see if they are using a motor to extend the spring
    }

    public void clampHookTwo(){ //b button
        //same as clampHookOne
    }

    public void holdArm(){
        rung1.set(0);
    }

    public void holdRotator(){
        //set rotational motor to zero
    }
}
