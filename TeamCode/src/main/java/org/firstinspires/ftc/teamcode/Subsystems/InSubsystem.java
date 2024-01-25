package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class InSubsystem extends SubsystemBase {
    Motor inSlideL, inSlideR;
    public InSubsystem(Motor inSlideL, Motor inSlideR) {
        this.inSlideL = inSlideL;
        this.inSlideR = inSlideR;
    }

    public void setPower(double power) {
        WriteSubsystem.motorNewPower.put(inSlideL, power);
        WriteSubsystem.motorNewPower.put(inSlideR, power);
    }

    public void setPosition(int position) {
        //TODO: add pid math here
    }
}
