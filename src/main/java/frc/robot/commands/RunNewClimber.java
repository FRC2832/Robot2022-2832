package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberNew;

public class RunNewClimber extends CommandBase {
    private final ClimberNew climberNew;
    private final XboxController controller;

    public RunNewClimber(ClimberNew climberNew, XboxController controller) {
        super();
        this.climberNew = climberNew;
        this.controller = controller;
        addRequirements(climberNew);
    }

    @Override
    public void execute() {
        if (!climberNew.getUnlocked()) { // if not unlocked the climberNew won't move
            climberNew.stopArm();
            return;
        }

        if (controller.getRightX() > 0.2 && controller.getRightX() < 0.6) { // counter clockwise slow
            climberNew.armCounterclockwise(true);
        } else if (controller.getRightX() >= 0.6) { //counter fast
            climberNew.armCounterclockwise(false);
        } else if (controller.getRightX() < -0.2 && controller.getRightX() > -0.6) { //clockwise slow
            climberNew.armClockwise(true);
        } else if (controller.getRightX() <= -0.6) { // clockwise fast
            climberNew.armClockwise(false);
        } else {
            climberNew.stopArm();
        }

    }

    @Override
    public void end(boolean interrupted) {
        climberNew.stopArm();
    }

}
