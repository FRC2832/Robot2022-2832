package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Climber;
import frc.robot.Ingestor;
import frc.robot.Shooter;

public class RunClimber extends CommandBase {
    private final Climber climber;
    private final XboxController controller;
    private final Ingestor ingestor;

    public RunClimber(Climber climber, Ingestor ingestor, XboxController controller) {
        this.climber = climber;
        this.controller = controller;
        this.ingestor = ingestor;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        int pov = controller.getPOV();
        if(pov != -1) {
            Shooter.setCoast(false);
        }
        switch (pov) {
            case 180: case 135: case 225: // Down pressed
                climber.arm1Down();
                break;
            case 0: case 45: case 315: // Up pressed
                // ingestor.liftIngestor();
                climber.arm1Up();
                break;
            case 90:
                climber.arm2Up();
            case 270:
                climber.arm2Down();
            default: // Nothing pressed. Move neither arm.
                climber.arm1Hold();
                climber.arm2Hold();
                break;
        }
    }

}
