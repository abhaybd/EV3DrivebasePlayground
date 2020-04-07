package com.coolioasjulio.ev3;

import trclib.TrcDriveBase;
import trclib.TrcMotor;
import trclib.TrcUtil;

import java.util.LinkedList;
import java.util.Queue;

public class DifferentialDriveBase extends TrcDriveBase {

    private static final int ROT_VEL_SMOOTH_WINDOW = 6;

    private TrcMotor leftMotor, rightMotor;
    private double trackWidth;
    private double lastLeftPos, lastRightPos;
    private double inchesPerTick;
    private Double lastTime;
    private Queue<double[]> headingQueue;

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

        headingQueue = new LinkedList<>();

        new Thread(() -> {
            while (!Thread.interrupted()) {
                updateOdometry();
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
            }
        }).start();
    }

    @Override
    public void resetOdometry(boolean hardware, boolean resetHeading) {
        if (leftMotor != null) {
            leftMotor.resetPosition(hardware);
        }
        if (rightMotor != null) {
            rightMotor.resetPosition(hardware);
        }
        xPos = 0.0;
        yPos = 0.0;
        xVel = 0.0;
        yVel = 0.0;
        if (resetHeading) {
            heading = 0.0;
        }
        rotVel = 0.0;
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

            double dTheta = 0; //(left - right) / trackWidth; // in radians
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
            headingQueue.add(new double[]{currTime, heading});
            while (headingQueue.size() > ROT_VEL_SMOOTH_WINDOW) {
                headingQueue.remove();
            }
            if (headingQueue.size() == ROT_VEL_SMOOTH_WINDOW) {
                double[] prevHeading = headingQueue.remove();
                rotVel = (heading - prevHeading[1]) / (currTime - prevHeading[0]);
            } else {
                rotVel = 0;
            }
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
