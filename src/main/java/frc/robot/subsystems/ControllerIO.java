package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public final class ControllerIO {
    private final XboxController driverController;
    private final XboxController operatorController;

    private ControllerIO(int driverPort, int operatorPort) {
        driverController = new XboxController(driverPort);
        operatorController = new XboxController(operatorPort);
    }

    public static ControllerIO getInstance() {
        return InstanceHolder.INSTANCE;
    }

    public XboxController getDriverController() {
        return driverController;
    }

    public XboxController getOperatorController() {
        return operatorController;
    }

    public boolean isDrivePadConnected() {
        return driverController.isConnected();
    }

    public boolean isDriveABtnPressed() {
        return driverController.getAButtonPressed();
    }

    public boolean isOpAButtonPressed() {
        return operatorController.getAButtonPressed();
    }

    public void stopAllRumble() {
        rumbleDriveController(0.0);
        rumbleOpController(0.0);
    }

    public void rumbleDriveController(double rumbleSpeed) {
        rumbleController(driverController, rumbleSpeed);
    }

    public void rumbleOpController(double rumbleSpeed) {
        rumbleController(operatorController, rumbleSpeed);
    }

    private static void rumbleController(XboxController controller, double rumbleSpeed) {
        controller.setRumble(RumbleType.kLeftRumble, rumbleSpeed);
        controller.setRumble(RumbleType.kRightRumble, rumbleSpeed);
    }

    private static final class InstanceHolder {
        private static final ControllerIO INSTANCE = new ControllerIO(0, 2);
    }
}
