package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ShooterOff;

public class Climber {
    private WPI_TalonFX rung1;
    private WPI_TalonFX rung23;
    private XboxController operatorController;
    private final Ingestor ingestor;
    private final Shooter shooter;
    private final double TRIGGER_SENSITIVITY = 0.5;
    // private final double CLIMBER_SPEED = 0.4;
    private final double CLIMBER_UP_SPEED = 0.6;
    private final double CLIMBER_DOWN_SPEED = 0.4;

    public Climber(Shooter shooter, Ingestor ingestor) {
        this.shooter = shooter;
        this.ingestor = ingestor;
        rung1 = new WPI_TalonFX(33);
        rung23 = new WPI_TalonFX(22);
        operatorController = new XboxController(2);
    }

    public void runClimber() {
        int pov = operatorController.getPOV();
        switch (pov) {
            case 180:
            case 135:
            case 225: // Down pressed. retract both arms.
                rung1.set(CLIMBER_DOWN_SPEED);
                break;
            case 0:
            case 45:
            case 315:// Up pressed. Extend both arms.
                // ingestor.liftIngestor();
                // TODO: the shooter motor is not turning off
                shooter.setDefaultCommand(new ShooterOff(shooter));
                rung1.set(-CLIMBER_UP_SPEED);
                break;
            default: // left, right, or nothing pressed. Move neither arm.
                rung1.set(0.0);
                break;
        }
        /*
         * if (operatorController.getPOV()) { // raise center arm
         * }
         * 
         * else if (operatorController.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY) { //
         * lower center arm
         * 
         * }
         * 
         * else {
         * rung1.set(0);
         * }
         */
        if (operatorController.getRightBumper()) { // raise double arm
            rung23.set(CLIMBER_UP_SPEED);
        }

        else if (operatorController.getRightTriggerAxis() >= TRIGGER_SENSITIVITY) { // lower double arm
            rung23.set(CLIMBER_DOWN_SPEED);
        }

        else {
            rung23.set(0);
        }
    }

    /*
     * public void raiseRung1Arm(){
     * rung1.set(ControlMode.Velocity, 1000);
     * }
     * public void lowerRung1Arm(){
     * rung1.set(ControlMode.Velocity, -1000);
     * }
     * public void raiseRung2Arm(){
     * rung23.set(ControlMode.Velocity, 1000);
     * }
     * public void lowerRung2Arm(){
     * rung23.set(ControlMode.Velocity, -1000);
     * }
     */
}
