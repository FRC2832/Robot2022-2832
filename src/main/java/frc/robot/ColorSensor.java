package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

public class ColorSensor {
    private ColorSensorV3 colorSensor;
    private Color color;
    private ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = new Color(0.171, 0.421, 0.406);
    private final Color kRedTarget = new Color(0.499, 0.362, 0.138);
    private final Color kUnknownTarget = new Color(0.269, 0.481, 0.249);
    private CargoColor colorMatch;
    // private DigitalInput proxSensor;

    public enum CargoColor {
        Red,
        Blue,
        Unknown
    }

    public ColorSensor(){
        I2C.Port port = I2C.Port.kOnboard; // TODO: Need to verify this.
        colorSensor = new ColorSensorV3(port);

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kUnknownTarget);
        colorMatch = CargoColor.Unknown;
        color = Color.kBlack;

        // proxSensor = new DigitalInput(0);
    }

    public CargoColor getColorSensor() {
        return colorMatch;
    }

    /*
    public boolean getProxSensor() {
        return !proxSensor.get();
    }
    */

    public void runColorSensor(){
        color = colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(color);
        if (match.color == kBlueTarget) {
            System.out.println("BLUE");
            colorMatch = CargoColor.Blue;
        } else if (match.color == kRedTarget) {
            System.out.println("RED");
            colorMatch = CargoColor.Red;
        } else {
            System.out.println("UNKNOWN");
            colorMatch = CargoColor.Unknown;
        }
    }
    
}