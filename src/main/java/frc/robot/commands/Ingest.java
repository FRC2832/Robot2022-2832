package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Ingestor;


public class Ingest extends CommandBase {
    private Timer timer;
    private Ingestor ingestor;

    private static final double INGESTOR_SPEED = 0.75; // 1000.0;
    private static final double STAGE_1_SPEED = 0.75;// 1000.0;
    private static final double STAGE_2_SPEED = 0.75; // 1000.0;
    private static final double INGESTOR_LIFT_SPEED = 0.25;

    public Ingest(Ingestor ingestor) {

        this.ingestor = ingestor;
        addRequirements(ingestor);

    }

    public void initialize(){
        timer.start();
    }

    @Override
    public void execute() {

        ingestor.getIngestorWheels().set(-INGESTOR_SPEED);
        ingestor.getStage1Conveyor().set(STAGE_1_SPEED);
    }

    @Override
    public boolean isFinished(){
        return (timer.get() > 5 || ingestor.getStage1Proximity());
    }

}
