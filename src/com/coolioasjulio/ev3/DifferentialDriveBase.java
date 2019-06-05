package com.coolioasjulio.ev3;

import trclib.TrcDriveBase;
import trclib.TrcMotor;
import trclib.TrcUtil;

public class DifferentialDriveBase extends TrcDriveBase {
    private TrcMotor leftMotor, rightMotor;
    private double trackWidth;
    private double lastLeftPos, lastRightPos;
    private double inchesPerTick;
    private Double lastTime;

    public DifferentialDriveBase(TrcMotor leftMotor, TrcMotor rightMotor, double trackWidth, double inchesPerTick) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.trackWidth = trackWidth;
        this.inchesPerTick = inchesPerTick;
        this.setPositionScales(1, 1, 1);
    }

    @Override
    protected void updateOdometry(Odometry odometry) {
        double currTime = TrcUtil.getCurrentTime();
        if (lastTime != null) {
            double leftPos = leftMotor.getPosition();
            double rightPos = rightMotor.getPosition();
            double left = leftPos * inchesPerTick - lastLeftPos;
            double right = rightPos * inchesPerTick - lastRightPos;
            lastLeftPos = leftPos;
            lastRightPos = rightPos;

            double dTheta = (left - right) / trackWidth; // in radians
            double turningRadius = (trackWidth / 2) * (left + right) / (left - right);
            double dx, dy;
            if (Double.isFinite(turningRadius)) {
                dx = turningRadius * (1 - Math.cos(dTheta));
                dy = turningRadius * Math.sin(dTheta);
            } else {
                dx = 0.0;
                dy = TrcUtil.average(left, right);
            }

            double headingRad = Math.toRadians(odometry.gyroHeading);
            double x = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
            double y = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);
            odometry.xRawPos += x;
            odometry.yRawPos += y;

            double dt = currTime - lastTime;
            odometry.xRawVel = x / dt;
            odometry.yRawVel = y / dt;

            double dThetaDeg = Math.toDegrees(dTheta);
            odometry.gyroHeading += dThetaDeg;
            odometry.gyroTurnRate = dThetaDeg / dt;
        }
        lastTime = currTime;
    }

    @Override
    public void tankDrive(double leftPower, double rightPower, boolean inverted) {
        if (inverted) {
            double temp = leftPower;
            leftPower = rightPower;
            rightPower = temp;
        }

        leftPower /= inchesPerTick;
        rightPower /= inchesPerTick;

        leftMotor.set((float) leftPower);
        rightMotor.set((float) rightPower);
    }
}
