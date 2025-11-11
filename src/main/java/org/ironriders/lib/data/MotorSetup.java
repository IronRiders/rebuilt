package org.ironriders.lib.data;

public class MotorSetup {

    public MotorSetup(int motorId, boolean InversionStatus) {
        this.motorId = motorId;
        this.InversionStatus = InversionStatus;
    }

    public int getId() {
        return motorId;
    }

    public boolean getInversionStatus() {
        return InversionStatus;
    }

    public int motorId;
    public boolean InversionStatus;
}
