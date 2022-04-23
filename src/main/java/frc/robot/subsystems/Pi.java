package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pi extends SubsystemBase {
    private static final double CAM_X_RES = 640.0;
    private static final Number[] DEFAULT_NUMBER_VALUE = new Number[0];
    // private final double CAM_Y_RES = 480;
    // public final double TARGET_CENTER_X = 320.0;
    private static boolean isCargoCamConnected;
    private static boolean isTargetCamConnected;
    private static boolean targetMoveRight;
    private static boolean targetMoveLeft;
    private static boolean cargoMoveRight;
    private static boolean cargoMoveLeft;
    private static double targetCenterYOutput;
    private static double targetCenterXOutput;
    private static double cargoCenterXOutput;
    private static double cargoCenterYOutput;
    private static double oldTargetY;
    private final NetworkTableInstance netTableInstance = NetworkTableInstance.getDefault();
    private final NetworkTableEntry cargoCenterX;
    private final NetworkTableEntry cargoCenterY;
    private final NetworkTableEntry allianceColor;
    private final NetworkTableEntry targetCenterX;
    private final NetworkTableEntry targetCenterY;
    private final NetworkTableEntry targetWidth;
    private final NetworkTableEntry targetHeight;
    private final NetworkTableEntry targetArea;
    private final NetworkTableEntry cargoCameraConnected;
    private final NetworkTableEntry targetCameraConnected;
    private final PIDController pid;
    private Number[] cargoCenterXArray;
    private Number[] cargoCenterYArray;
    private Number[] targetCenterXArray;
    private Number[] targetCenterYArray;
    private Number[] targetWidthArray;
    private Number[] targetHeightArray;
    private Number[] targetAreaArray;
    private int targetLostCounter;
    //private double turnMotorVal;
    //private double scaledTurnMotorVal;

    public Pi() {
        super();
        //NetworkTableInstance netTableInstance = NetworkTableInstance.getDefault();
        NetworkTable table = netTableInstance.getTable("vision");
        cargoCenterX = table.getEntry("cargoX");
        cargoCenterY = table.getEntry("cargoY");
        allianceColor = table.getEntry("alliance");
        targetCenterX = table.getEntry("targetX");
        targetCenterY = table.getEntry("targetY");
        targetWidth = table.getEntry("targetWidth");
        targetHeight = table.getEntry("targetHeight");
        targetArea = table.getEntry("targetArea");
        cargoCameraConnected = table.getEntry("cargoCamConnected");
        targetCameraConnected = table.getEntry("targetCamConnected");
        targetCenterYOutput = -1;
        pid = new PIDController(0.2, 0, 0); // values from tyros last year were 0.35, 0.05, 0.8
        pid.setSetpoint(CAM_X_RES / 2);
        pid.setTolerance(10); // tolerance of 10 pixels
    }

    public static boolean isCargoCentered() {
        return !(cargoMoveRight || cargoMoveLeft);
    }

    // if true, the robot needs to turn right to see the cargo
    public static boolean getCargoMoveRight() {
        return cargoMoveRight;
    }

    // if true, the robot needs to turn left to see the cargo
    public static boolean getCargoMoveLeft() {
        return cargoMoveLeft;
    }

    public static double getCargoCenterX() {
        return cargoCenterXOutput;
    }

    public static double getCargoCenterY() {
        return cargoCenterYOutput;
    }

    public static boolean isTargetCentered() {
        return !(targetMoveRight || targetMoveLeft);
    }

    // if true, the robot needs to turn right to see the target
    public static boolean getTargetMoveRight() {
        return targetMoveRight;
    }

    // if true, the robot needs to turn left to see the target
    public static boolean getTargetMoveLeft() {
        return targetMoveLeft;
    }

    static double getTargetCenterY() {
        return targetCenterYOutput;
    }

    public static double getTargetCenterX() {
        return targetCenterXOutput;
    }

    public static boolean getIsCargoCamConnected() {
        return isCargoCamConnected;
    }

    public static boolean getIsTargetCamConnected() {
        return isTargetCamConnected;
    }

    @Override
    public void periodic() {
        isCargoCamConnected =
                cargoCameraConnected.getBoolean(true); // TODO: Change default to false when it's added to Python code.
        if (isCargoCamConnected) {
            processCargo();
        } else {
            cargoMoveRight = false;
            cargoMoveLeft = false;
            cargoCenterXOutput = -1;
            cargoCenterYOutput = -1;
        }
        isTargetCamConnected =
                targetCameraConnected.getBoolean(true); // TODO: Change default to false when it's added to Python code.
        if (isTargetCamConnected) {
            processTargets();
        } else {
            targetMoveRight = false;
            targetMoveLeft = false;
            if (targetLostCounter > 4) { // keep last target data for 5 loops ~ less than a second
                targetCenterYOutput = -1;
                targetCenterXOutput = -1;
                // System.out.println("lost vision");
            } else {
                //System.out.println("saving old vision target for " + targetLostCounter + " loops"); // TODO:
                // Comment this out before comp!
                targetLostCounter++;
            }
        }
        // centerToTarget(); // take this out when done testing
    }

    private void processCargo() {
        cargoCenterXArray = cargoCenterX.getNumberArray(DEFAULT_NUMBER_VALUE);
        cargoCenterYArray = cargoCenterY.getNumberArray(DEFAULT_NUMBER_VALUE);
        if (cargoCenterXArray.length == 0 || cargoCenterYArray.length == 0) {
            cargoMoveRight = false;
            cargoMoveLeft = false;
            cargoCenterXOutput = -1;
            cargoCenterYOutput = -1;
            return;
        }
        sortCargo();
        // take the cargo with the largest y value (closest to the robot)
        double cargoX = cargoCenterXArray[cargoCenterXArray.length - 1].doubleValue();
        cargoCenterXOutput = cargoX;
        cargoCenterYOutput = cargoCenterYArray[cargoCenterYArray.length - 1].doubleValue();
        if (cargoX < (CAM_X_RES / 2) - (CAM_X_RES * 0.05)) {
            cargoMoveRight = false;
            cargoMoveLeft = true;
        } else if (cargoX > (CAM_X_RES / 2) + (CAM_X_RES * 0.05)) {
            cargoMoveLeft = false;
            cargoMoveRight = true;
        } else {
            cargoMoveRight = false;
            cargoMoveLeft = false;
        }
    }

    private void processTargets() {
        targetCenterXArray = targetCenterX.getNumberArray(DEFAULT_NUMBER_VALUE);
        targetCenterYArray = targetCenterY.getNumberArray(DEFAULT_NUMBER_VALUE);
        targetWidthArray = targetWidth.getNumberArray(DEFAULT_NUMBER_VALUE);
        targetHeightArray = targetHeight.getNumberArray(DEFAULT_NUMBER_VALUE);
        targetAreaArray = targetArea.getNumberArray(DEFAULT_NUMBER_VALUE);
        int size = targetCenterXArray.length;
        // check if vision saw a target
        if (size == 0) {
            targetMoveRight = false;
            targetMoveLeft = false;
            if (targetLostCounter > 4) { // keep last target data for 5 loops ~ less than a second
                targetCenterYOutput = -1;
                targetCenterXOutput = -1;
                // System.out.println("lost vision");
            } else {
                //System.out.println("saving old vision target for " + targetLostCounter + " loops"); // TODO:
                // Comment this out before comp!
                targetLostCounter++;
            }
            return;
        }
        targetLostCounter = 0;
        // consistency check
        if (size == targetCenterYArray.length && size == targetWidthArray.length && size == targetHeightArray.length &&
            size == targetAreaArray.length) {
            sortTargets();
        } else {
            // unknown order, skip this loop
            return;
        }

        // pick a target just right of center so the cargo hopefully doesn't bounce out
        int index = 0;
        if (size > 1) {
            index = (int) Math.ceil(size / 2.0);
        }
        double targetX =
                targetCenterXArray[index].doubleValue(); // TODO: Is there less overhang with casting or calling
        // doubleValue()?
        targetCenterYOutput = targetCenterYArray[index].doubleValue();
        if (Math.abs(targetCenterYOutput - oldTargetY) ==
            0.5) { // changes of only 0.5 pixels to Y are not useful // TODO: Equality comparisons with floating
            // point numbers is rarely a good idea.
            targetCenterYOutput = oldTargetY;
        } else {
            oldTargetY = targetCenterYOutput;
        }
        targetCenterXOutput = targetX;
        if (targetX < ((CAM_X_RES / 2) - (CAM_X_RES * 0.05))) {
            targetMoveRight = false;
            targetMoveLeft = true;
        } else if (targetX > ((CAM_X_RES / 2) + (CAM_X_RES * 0.05))) {
            targetMoveLeft = false;
            targetMoveRight = true;
        } else {
            targetMoveRight = false;
            targetMoveLeft = false;
        }
    }

    private void sortCargo() {
        int size = cargoCenterXArray.length;
        for (int i = 1; i < size; i++) {
            double keyX = cargoCenterXArray[i].doubleValue();
            double keyY = cargoCenterYArray[i].doubleValue();
            int j = i - 1;
            while (j >= 0 && cargoCenterYArray[j].doubleValue() > keyY) {
                cargoCenterXArray[j + 1] = cargoCenterXArray[j];
                cargoCenterYArray[j + 1] = cargoCenterYArray[j];
                j--;
            }
            cargoCenterXArray[j + 1] = keyX;
            cargoCenterYArray[j + 1] = keyY;
        }
    }

    private void sortTargets() {
        int size = targetCenterXArray.length;
        for (int i = 1; i < size; i++) {
            double keyX = targetCenterXArray[i].doubleValue();
            double keyY = targetCenterYArray[i].doubleValue();
            double keyH = targetHeightArray[i].doubleValue();
            double keyW = targetWidthArray[i].doubleValue();
            double keyA = targetAreaArray[i].doubleValue();
            int j = i - 1;
            while (j >= 0 && targetCenterXArray[j].doubleValue() > keyX) {
                targetCenterXArray[j + 1] = targetCenterXArray[j];
                targetCenterYArray[j + 1] = targetCenterYArray[j];
                targetHeightArray[j + 1] = targetHeightArray[j];
                targetWidthArray[j + 1] = targetWidthArray[j];
                targetAreaArray[j + 1] = targetAreaArray[j];
                j--;
            }
            targetCenterXArray[j + 1] = keyX;
            targetCenterYArray[j + 1] = keyY;
            targetHeightArray[j + 1] = keyH;
            targetWidthArray[j + 1] = keyW;
            targetAreaArray[j + 1] = keyA;
        }
    }

    public void centerToTarget() {
        double pidVal = pid.calculate(targetCenterXOutput);
        // double maxTurnSpeed = 90; // in degrees, converted to radians at the end
        // double minTurnSpeed = 10; // TODO: what actually is the min speed?

        // if (motorVal > 0.3) {
        //     motorVal = 0.3;
        // } else if (motorVal < -0.3) {
        //     motorVal = -0.3;
        // }

        // TODO: add motor commands (remember to convert to radians)


        double positionError = pid.getPositionError();
        SmartDashboard.putNumber("Centering PID error", positionError);
        SmartDashboard.putNumber("Centering PID value", pidVal);

    }

    // sends alliance color to the python code so it knows what color cargo to look
    // for
    public void sendAlliance() {
        Alliance alliance = DriverStation.getAlliance();
        String color = "blue";
        if (alliance == Alliance.Red) {
            color = "red";
        }
        allianceColor.setString(color);
    }

}
