package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ClimberOld;
import frc.robot.Shooter;

public class RunClimber extends CommandBase {
    private final ClimberOld climber;
    private final Shooter shooter;
    private final XboxController controller;

    public RunClimber(ClimberOld climber, XboxController controller, Shooter shooter) {
        this.climber = climber;
        this.shooter = shooter;
        this.controller = controller;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        int pov = controller.getPOV();
        if (pov != -1) {
            Shooter.setCoast(false);
            shooter.setHoodAngle(20.0);
        }
        switch (pov) {
            case 180: case 135: case 225: // Down pressed
                climber.arm1Down();
                break;
            case 0: case 45: case 315: // Up pressed
                // ingestor.liftIngestor();
                climber.arm1Up();
                break;
            /*case 90:
                climber.arm2Up();
                break;
            case 270:
                climber.arm2Down();
                break;*/
            default: // Nothing pressed. Move neither arm.
                climber.arm1Hold();
                //climber.arm2Hold();
                break;
        }
    }

}
