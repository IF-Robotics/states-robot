package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import java.util.ArrayList;
import java.util.HashMap;

public class WriteSubsystem extends SubsystemBase {
//    public static
//    ArrayList<Motor> motors;
    private HashMap<Motor, Double> motorCurrentPower;
    public static HashMap<Motor, Double> motorNewPower;
    double tempPower = 0;

    public WriteSubsystem(ArrayList<Motor> motors) {
        for(Motor m: motors) {
            motorCurrentPower.put(m, 0.0);
            motorNewPower.put(m, 0.0);
        }
    }

    @Override
    public void periodic() {
        for(Motor m: motorCurrentPower.keySet()) {
            tempPower = Math.round(motorNewPower.get(m) * 1000) / 1000;
            if(tempPower != motorCurrentPower.get(m)) {
                m.set(tempPower);
                motorCurrentPower.put(m, tempPower);
            }
        }
    }
}
