package frc.robot;

public class Diagnostics {
    private REVDigitBoard digit;
    int loops;  //temp variable to display loops on the digit board
    DiagState state;

    enum DiagState {
        Menu,
        SensorInit,
        SensorCheck,
        ActiveCheck
    }

    public Diagnostics(REVDigitBoard digit) {
        this.digit = digit;
        state = DiagState.Menu;
    }

    boolean lastAPress = false;

    public void periodic() {
        boolean aReleased = false;
        //check for falling edge
        if(digit.getButtonA() == false && lastAPress == true) {
            aReleased = true;
        }
        lastAPress = digit.getButtonA();
        double value = digit.getPot();
        String disp = "";

        if(state == DiagState.Menu) {
            if (value < 1.6667) {
                disp = "Actv";
            } else if(value < 3.3333) {
                disp = "Sens";
            } else {
                disp = "Init";
            }

            //if a is released, transition to new state
            if(aReleased==true) {

            }
        }
        
        //display something on the digit board
        //digit.display("Rony");
        //digit.display(digit.getPot());
        digit.display(disp);
        loops++;
    }
}
