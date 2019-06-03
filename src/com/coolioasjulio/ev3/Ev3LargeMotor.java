package com.coolioasjulio.ev3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import trclib.TrcMotor;

public class Ev3LargeMotor extends TrcMotor {

    private EV3LargeRegulatedMotor motor;
    private String instanceName;
    private boolean inverted = false;
    private boolean posInverted = false;

    public Ev3LargeMotor(String instanceName, EV3LargeRegulatedMotor motor) {
        super(instanceName);
        this.instanceName = instanceName;
        this.motor = motor;
    }

    @Override
    public double getMotorPosition() {
        return motor.getPosition() * (posInverted ? -1 : 1);
    }

    @Override
    public void setMotorPower(double v) {
        motor.setSpeed((float) (motor.getMaxSpeed() * (inverted ? -v : v)));
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public double getPower() {
        return motor.getSpeed() / motor.getMaxSpeed();
    }

    @Override
    public boolean isLowerLimitSwitchActive() {
        return false;
    }

    @Override
    public boolean isUpperLimitSwitchActive() {
        return false;
    }

    @Override
    public void resetPosition(boolean b) {
        motor.resetTachoCount();
    }

    @Override
    public void setBrakeModeEnabled(boolean b) {
        System.err.printf("Motor %s does not support configuring brake mode!\n", instanceName);
    }

    @Override
    public void setInverted(boolean b) {
        inverted = b;
    }

    @Override
    public void setPositionSensorInverted(boolean b) {
        posInverted = b;
    }

    @Override
    public void setSoftLimitEnabled(boolean b, boolean b1) {

    }

    @Override
    public void setSoftLowerLimit(double v) {

    }

    @Override
    public void setSoftUpperLimit(double v) {

    }
}
