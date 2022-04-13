package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;

public class Ingest extends CommandBase {
    private static final double INGESTOR_SPEED = 0.75; // 1000.0;
    private static final double STAGE_1_SPEED = 0.75;// 1000.0;
    private static final double STAGE_2_SPEED = 0.75; // 1000.0;
    private final Ingestor ingestor;
    private static final Timer TIMER = new Timer();
    // private static final double INGESTOR_LIFT_SPEED = 0.25;

    public Ingest(Ingestor ingestor) {
        this.ingestor = ingestor;
        addRequirements(ingestor);
    }

    @Override
    public void initialize() {
        TIMER.reset();
        TIMER.start();
    }

    @Override
    public void execute() {

        ingestor.getIngestorWheels().set(-INGESTOR_SPEED);
        ingestor.getStage1Conveyor().set(STAGE_1_SPEED);
        ingestor.getStage2Conveyor().set(STAGE_2_SPEED);
    }

    @Override
    public boolean isFinished() {
        return (TIMER.get() > 5 || ingestor.getStage1Proximity());
    }

    @Override
    public void end(boolean interrupted) {
        TIMER.stop();
    }

}
