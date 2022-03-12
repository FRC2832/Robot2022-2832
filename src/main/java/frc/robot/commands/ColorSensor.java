package frc.robot.commands;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

public class ColorSensor {
    private ColorSensorV3 stage2ColorSensor;
    private Color detectedColor;
    private String color;
    
    public ColorSensor(){
        I2C.Port port = I2C.Port.kOnboard; // TODO: Need to verify this.
        stage2ColorSensor = new ColorSensorV3(port);
        color = "";
    }

    public void runColorSensor(){
        detectedColor = stage2ColorSensor.getColor();
        if(stage2ColorSensor.getProximity() < 1000){ // 0-10 cm range; max .getProximity() value = 2470
            System.out.println("getProximity() < 1000");
            
            if(detectedColor.red > 128.0){
                System.out.println("Red detected @" + detectedColor.red);
                color = "red";
            }
            else if(detectedColor.blue > 128.0){
                System.out.println("Blue detected @" + detectedColor.blue);
                color = "blue";
            }
            else{
                System.out.println("Blue/Red not detected");
            }
        }
        else{
            System.out.println("getProximity >= 1000");
        }

    }

    public String getColor(){
        return color;
    }
}