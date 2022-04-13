package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final double CLIMBER_SLOW_SPEED = 0.4;
    private final double CLIMBER_FAST_SPEED = 0.7;
    private final WPI_TalonFX motorA;
    //private final WPI_TalonFX motorB;
    private final Ingestor ingestor;
    private final XboxController controller;
    private boolean unlocked;

    public Climber(Ingestor ingestor, XboxController controller) {
        motorA = new WPI_TalonFX(CanIDConstants.CLIMB_A);
        // motorB = new WPI_TalonFX(CanIDConstants.CLIMB_B);
        motorA.setNeutralMode(NeutralMode.Brake);
        // motorB.setNeutralMode(NeutralMode.Brake);
        this.ingestor = ingestor;
        this.controller = controller;
        unlocked = false;
    }

    @Override
    public void periodic() {
        if (controller.getRightStickButton()) {
            Robot.rumbleController(controller, 1);
            unlocked = true;
        } else {
            Robot.stopControllerRumble(controller);
        }
    }

    public void armCounterclockwise(boolean slow) {
        if (slow) {
            motorA.set(CLIMBER_SLOW_SPEED);
            // motorB.set(CLIMBER_SLOW_SPEED);
        } else {
            motorA.set(CLIMBER_FAST_SPEED);
            // motorB.set(CLIMBER_FAST_SPEED);
        }
    }

    public void armClockwise(boolean slow) {
        if (slow) {
            motorA.set(-CLIMBER_SLOW_SPEED);
            // motorB.set(-CLIMBER_SLOW_SPEED);
        } else {
            motorA.set(-CLIMBER_FAST_SPEED);
            // motorB.set(-CLIMBER_FAST_SPEED);
        }
    }

    public void stopArm() {
        motorA.set(0);
        // motorB.set(0);
    }

    public boolean getUnlocked() {
        return unlocked;
    }

    public void setUnlocked(boolean unlocked) {
        this.unlocked = unlocked;
    }
}
