package frc.robot;

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

    public Pi() {
        netTableInstance = NetworkTableInstance.getDefault();
        table = netTableInstance.getTable("vision");
        cargoCenterX = table.getEntry("cargoX");
        cargoCenterY = table.getEntry("cargoY");
        allianceColor = table.getEntry("alliance");
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
        System.out.println("cargo x: " + cargoCenterXArray);
        System.out.println("cargo y: " + cargoCenterYArray);
    }
}
