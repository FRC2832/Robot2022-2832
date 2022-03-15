package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Ingestor;

public class RunIngestor extends CommandBase {
    private final Ingestor ingestor;
    private static final double TRIGGER_SENSITIVITY = 0.5;
    private XboxController controller;

    public RunIngestor(Ingestor ingestor, XboxController controller) {
        this.ingestor = ingestor;
        this.controller = controller;
    }

    @Override
    public void execute() {
        if(controller.getRightTriggerAxis() >= TRIGGER_SENSITIVITY){ 
            ingestor.intake();
        } else if(controller.getLeftTriggerAxis() >= TRIGGER_SENSITIVITY){ 
            ingestor.outtake();
        } else {
            ingestor.holdWheels();
        }

        if (controller.getXButton()) { 
            ingestor.stage2Forward();
        } else if (controller.getYButton()) {
            ingestor.stage2Back();
        } else {
            ingestor.stage2Hold();
        }

        if (controller.getBButton()) { 
            ingestor.liftIngestor();
        } else if (controller.getAButton()) { 
            ingestor.lowerIngestor();
        } else {
            ingestor.holdIngestor();
        }
    }

}
