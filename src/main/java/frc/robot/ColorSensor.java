package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private static final Color BLUE_TARGET = new Color(0.171, 0.421, 0.406);
    private static final Color RED_TARGET = new Color(0.499, 0.362, 0.138);
    private static final Color UNKNOWN_TARGET = new Color(0.269, 0.481, 0.249);
    private static ColorSensorV3 colorSensor;
    private static CargoColor cargoColor;
    private final ColorMatch colorMatcher = new ColorMatch();
    private Color color;
    // private DigitalInput proxSensor;

    public ColorSensor() {
        I2C.Port port = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(port);

        colorMatcher.addColorMatch(BLUE_TARGET);
        colorMatcher.addColorMatch(RED_TARGET);
        colorMatcher.addColorMatch(UNKNOWN_TARGET);
        cargoColor = CargoColor.Unknown;
        color = Color.kBlack;

        // proxSensor = new DigitalInput(0);
    }

    public static CargoColor getCargoColor() {
        return cargoColor;
    }

    /*public static int getProx() {
        return colorSensor.getProximity();
    }*/

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

    /*
     * public boolean getProxSensor() {
     * return !proxSensor.get();
     * }
     */

    public enum CargoColor {
        Red, Blue, Unknown
    }

}