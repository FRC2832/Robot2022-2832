package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private static ColorSensorV3 colorSensor;
    private Color color;
    private ColorMatch colorMatcher = new ColorMatch();
    private final Color BLUE_TARGET = new Color(0.171, 0.421, 0.406);
    private final Color RED_TARGET = new Color(0.499, 0.362, 0.138);
    private final Color UNKNOWN_TARGET = new Color(0.269, 0.481, 0.249);
    private static CargoColor cargoColor;
    // private DigitalInput proxSensor;

    public enum CargoColor {
        Red,
        Blue,
        Unknown
    }

    public ColorSensor() {
        I2C.Port port = I2C.Port.kOnboard; // TODO: Need to verify this.
        colorSensor = new ColorSensorV3(port);

        colorMatcher.addColorMatch(BLUE_TARGET);
        colorMatcher.addColorMatch(RED_TARGET);
        colorMatcher.addColorMatch(UNKNOWN_TARGET);
        cargoColor = CargoColor.Unknown;
        color = Color.kBlack;

        // proxSensor = new DigitalInput(0);
    }

    public static int getProx() {
        return colorSensor.getProximity();
    }

    public static String getCargoColor() {
        return cargoColor.toString();
    }

    /*
     * public boolean getProxSensor() {
     * return !proxSensor.get();
     * }
     */

    @Override
    public void periodic() {
        color = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(color);
        if (match.color == BLUE_TARGET) {
            // System.out.println("BLUE");
            cargoColor = CargoColor.Blue;
        } else if (match.color == RED_TARGET) {
            // System.out.println("RED");
            cargoColor = CargoColor.Red;
        } else {
            // System.out.println("UNKNOWN");
            cargoColor = CargoColor.Unknown;
        }
    }

}