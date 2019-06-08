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

    private double xPos;
    private double yPos;
    private double xVel;
    private double yVel;
    private double heading;
    private double rotVel;

    public DifferentialDriveBase(TrcMotor leftMotor, TrcMotor rightMotor, double trackWidth, double inchesPerTick) {
        super(leftMotor, rightMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.trackWidth = trackWidth;
        this.inchesPerTick = inchesPerTick;
        this.setPositionScales(1, 1, 1);

        new Thread(() -> {
            while(!Thread.interrupted()) {
                updateOdometry();
            }
        }).start();
    }

    @Override
    protected void updateOdometry(Odometry odometry) {
    }

    @Override
    public double getRawYPosition() {
        return yPos;
    }

    @Override
    public double getYPosition() {
        return getRawYPosition();
    }

    @Override
    public double getRawXPosition() {
        return xPos;
    }

    @Override
    public double getXPosition() {
        return getRawXPosition();
    }

    @Override
    public double getYVelocity() {
        return yVel;
    }

    @Override
    public double getXVelocity() {
        return xVel;
    }

    @Override
    public double getHeading() {
        return heading;
    }

    @Override
    public double getGyroTurnRate() {
        return rotVel;
    }

    private void updateOdometry() {
        double currTime = TrcUtil.getCurrentTime();
        double leftPos = leftMotor.getMotorPosition();
        double rightPos = rightMotor.getMotorPosition();
        if (lastTime != null) {
            double left = inchesPerTick * (leftPos - lastLeftPos);
            double right = inchesPerTick * (rightPos - lastRightPos);

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

            double headingRad = Math.toRadians(heading);
            double x = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
            double y = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);
            xPos += x;
            yPos += y;

            double dt = currTime - lastTime;
            xVel = x / dt;
            yVel = y / dt;

            double dThetaDeg = Math.toDegrees(dTheta);
            heading += dThetaDeg;
            rotVel = dThetaDeg / dt;
        }
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        lastTime = currTime;
    }

    @Override
    public void tankDrive(double leftPower, double rightPower, boolean inverted) {
        if (inverted) {
            double temp = -leftPower;
            leftPower = -rightPower;
            rightPower = temp;
        }

        leftMotor.set(leftPower);
        rightMotor.set(rightPower);
    }
}
