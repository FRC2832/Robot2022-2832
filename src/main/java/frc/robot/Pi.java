package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Pi {
    private NetworkTableInstance netTableInstance;
    private NetworkTable table;
    private NetworkTableEntry cargoCenterX;
    private NetworkTableEntry cargoCenterY;
    private NetworkTableEntry allianceColor;
    private NetworkTableEntry targetCenterX;
    private NetworkTableEntry targetCenterY;
    private NetworkTableEntry targetWidth;
    private NetworkTableEntry targetHeight;
    private NetworkTableEntry targetArea;
    private Number[] targetCenterXArray;
    private Number[] targetCenterYArray;
    private Number[] targetWidthArray;
    private Number[] targetHeightArray;
    private Number[] targetAreaArray;
    private final double CAM_X_RES = 640;
    private final double CAM_Y_RES = 480;
    public final double TARGET_CENTER_X = 320;
    private static boolean targetMoveRight;
    private static boolean targetMoveLeft;
    private static boolean cargoMoveRight;
    private static boolean cargoMoveLeft;
    private double centerYOutput;
    private double centerXOutput;

    public Pi() {
        netTableInstance = NetworkTableInstance.getDefault();
        table = netTableInstance.getTable("vision");
        cargoCenterX = table.getEntry("cargoX");
        cargoCenterY = table.getEntry("cargoY");
        allianceColor = table.getEntry("alliance");
        targetCenterX = table.getEntry("targetX");
        targetCenterY = table.getEntry("targetY");
        targetWidth = table.getEntry("targetWidth");
        targetHeight = table.getEntry("targetHeight");
        targetArea = table.getEntry("targetArea");
        centerYOutput = -1;
    }

    // sends alliance color to the python code so it knows what color cargo to look for
    public void sendAlliance() {
        Alliance alliance = DriverStation.getAlliance();
        if(alliance == Alliance.Red) {
            allianceColor.setString("red");
        } else {
            allianceColor.setString("blue");
        }
    }

    public void processCargo() {
        Number[] cargoCenterXArray = cargoCenterX.getNumberArray(new Number[0]);
        Number[] cargoCenterYArray = cargoCenterY.getNumberArray(new Number[0]);
        if (cargoCenterXArray.length == 0) {
            cargoMoveRight = false;
            cargoMoveLeft = false;
            return;
        }
        // currently just taking the first cargo but considering taking the cargo with the largest y value becuase it should be closest to the robot
        double cargoX = (double) cargoCenterXArray[0];
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

    public void processTargets() {
        targetCenterXArray = targetCenterX.getNumberArray(new Number[0]);
        targetCenterYArray = targetCenterY.getNumberArray(new Number[0]);
        targetWidthArray = targetWidth.getNumberArray(new Number[0]);
        targetHeightArray = targetHeight.getNumberArray(new Number[0]);
        targetAreaArray = targetArea.getNumberArray(new Number[0]);
        int size = targetCenterXArray.length;
        //check if vision saw a target
        if (size == 0) {
            targetMoveRight = false;
            targetMoveLeft = false;
            centerYOutput = -1;
            centerXOutput = -1;
            return;
        }
        //consistency check
        if(  size == targetCenterYArray.length
          && size == targetWidthArray.length
          && size == targetHeightArray.length
          && size == targetAreaArray.length
          )
        {
            sortTargets();
        }
        else {
            //unknown order, skip this loop
            return;
        }

        // pick a target just right of center so the cargo hopefully doesn't bounce out
        int index = 0;
        if (size <= 1) {
            index = 0; 
        } else {
            index = (int) Math.ceil((double) size / 2);
        }
        double targetX = (double) targetCenterXArray[index];
        centerYOutput = (double) targetCenterYArray[index];
        centerXOutput = targetX;
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
    
    public void sortTargets() {
        int size = targetCenterXArray.length;
        for (int i = 1; i < size; ++i) {
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
                j = j - 1;
            }
            targetCenterXArray[j + 1] = keyX;
            targetCenterYArray[j + 1] = keyY;
            targetHeightArray[j + 1] = keyH;
            targetWidthArray[j + 1] = keyW;
            targetAreaArray[j + 1] = keyA;
        }
    }

    public static double LinearInterp(ArrayList<Pair<Double,Double>> list, double input) {
        //if input is smaller than the table, return the first element
        if(input < list.get(0).getFirst()) {
            return list.get(0).getSecond();
        }
        //if input is larger than the table, return the last element
        if(input > list.get(list.size()-1).getFirst()) {
            return list.get(list.size()-1).getSecond();
        }
        //otherwise the value is in the table
        for(int i=0; i<list.size()-1; i++) {
            double x0 = list.get(i).getFirst();
            double x1 = list.get(i+1).getFirst();
            if ((x0 <= input) && (input <= x1)) {
                //see https://en.wikipedia.org/wiki/Linear_interpolation
                double y0 = list.get(i).getSecond();
                double y1 = list.get(i+1).getSecond();
                return (y0*(x1-input) + y1*(input-x0))/(x1-x0);
            }
        }
        //should never happen...
        return Double.NaN;
    }

    public static boolean isCargoCentered() {
        return !cargoMoveRight && !cargoMoveLeft;
    }

    public static boolean getCargoMoveRight() {
        return cargoMoveRight;
    }

    public static boolean getCargoMoveLeft() {
        return cargoMoveLeft;
    }

    public static boolean isTargetCentered() {
        return !targetMoveRight && !targetMoveLeft;
    }

    public static boolean getTargetMoveRight() {
        return targetMoveRight;
    }

    public static boolean getTargetMoveLeft() {
        return targetMoveLeft;
    }

    public double getCenterY() {
        return centerYOutput;
    }

    public double getCenterX() {
        return centerXOutput;
    }
}
