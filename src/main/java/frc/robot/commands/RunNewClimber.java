package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Climber;

public class RunNewClimber extends CommandBase {
    private Climber climber;
    private XboxController controller;

    public RunNewClimber(Climber climber, XboxController controller) {
        this.climber = climber;
        this.controller = controller;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if(!climber.getUnlocked()) { // if not unlocked the climber won't move
            climber.stopArm();
            return;
        }

        if (controller.getRightX() > 0.2 && controller.getRightX() < 0.6) { // counter clockwise slow
            climber.armCounterclockwise(true);
        } else if (controller.getRightX() >= 0.6) { //counter fast
            climber.armCounterclockwise(false);
        } else if (controller.getRightX() < -0.2 && controller.getRightX() > -0.6) { //clockwise slow
            climber.armClockwise(true);
        } else if (controller.getRightX() <= -0.6) { // clockwise fast
            climber.armClockwise(false);
        } else {
            climber.stopArm();
        }

    }

    @Override
    public void end(boolean interrupted) {
        climber.stopArm();
    }

}
