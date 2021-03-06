package com.coolioasjulio.ev3;

import trclib.TrcMotor;
import trclib.TrcUtil;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class Ev3Motor extends TrcMotor {

    private static byte PORT_A = (byte) 0x01;
    private static byte PORT_B = (byte) 0x02;
    private static byte PORT_C = (byte) 0x04;
    private static byte PORT_D = (byte) 0x08;

    private static byte INPUT_PORT_A = (byte) 0x10;
    private static byte INPUT_PORT_B = (byte) 0x11;
    private static byte INPUT_PORT_C = (byte) 0x12;
    private static byte INPUT_PORT_D = (byte) 0x13;

    public enum Port {
        A(INPUT_PORT_A, PORT_A), B(INPUT_PORT_B, PORT_B), C(INPUT_PORT_C, PORT_C), D(INPUT_PORT_D, PORT_D);

        private byte input, output;

        Port(byte input, byte output) {
            this.output = output;
            this.input = input;
        }

        public byte getInput() {
            return input;
        }

        public byte getOutput() {
            return output;
        }
    }

    private static short encodeNumber(int num) {
        int header = 0b10000001 << 8;
        return (short) (header | (num & 0xFF));
    }

    public static void main(String[] args) throws Exception {
        Ev3Motor motor = new Ev3Motor("a", Port.C);
        motor.setBrakeModeEnabled(false);
        motor.resetPosition();

        System.out.println(motor.getMotorPosition());
        motor.setInverted(true);
        motor.setMotorPower(0.2);
        Thread.sleep(2000);
        motor.setMotorPower(0.0);
        System.out.println(motor.getMotorPosition());
        motor.resetPosition();
        motor.setInverted(false);
        motor.setMotorPower(0.2);
        Thread.sleep(2000);
        motor.setMotorPower(0.0);
        System.out.println(motor.getMotorPosition());
    }

    private Port port;
    private boolean inverted = false;
    private boolean posInverted = false;
    private double power = 0;
    private boolean brakeMode = false;
    private byte inputType;

    public Ev3Motor(String instanceName, Port port) {
        super(instanceName);
        this.port = port;
        this.inputType = getInputType();
    }

    private byte getInputType() {
        ByteBuffer command = ByteBuffer.allocateDirect(6);
        command.put((byte) 0x99);
        command.put((byte) 0x05);
        command.put((byte) 0x00);
        command.put(port.getInput());
        command.put((byte) 0x60);
        command.put((byte) 0x61);
        byte[] arr = Ev3Brick.getDefaultBrick().sendReplyCommand(command, 0, 2);
        return arr[0];
    }

    @Override
    public double getMotorPosition() {
        ByteBuffer command = ByteBuffer.allocateDirect(8);
        command.put((byte) 0x99);
        command.put((byte) 0x1c);
        command.put((byte) 0x00);
        command.put(port.getInput());
        command.put(inputType);
        command.put((byte) 0x00);
        command.put((byte) 0x01);
        command.put((byte) 0x60);
        byte[] arr = Ev3Brick.getDefaultBrick().sendReplyCommand(command, 0, 4);
        ByteBuffer buffer = ByteBuffer.allocateDirect(4);
        buffer.put(arr);
        return Integer.reverseBytes(buffer.getInt(0)) * (posInverted ? -1 : 1);
    }

    @Override
    public void resetPosition(boolean b) {
        ByteBuffer command = ByteBuffer.allocateDirect(3);
        command.put((byte) 0xb2);
        command.put((byte) 0x00);
        command.put(port.getOutput());
        Ev3Brick.getDefaultBrick().sendNoReplyCommand(command);
    }

    @Override
    public void setMotorPower(double v) {
        v = TrcUtil.clipRange(v);
        this.power = v;
        if (inverted) {
            v = -v;
        }
        if (v != 0.0) {
            int power = TrcUtil.round(v * 100);
            ByteBuffer command = ByteBuffer.allocateDirect(8);
            command.order(ByteOrder.BIG_ENDIAN); // it'll get flipped later
            command.put((byte) 0xa5);
            command.put((byte) 0x00);
            command.put(port.getOutput());
            command.putShort(encodeNumber(power));
            command.put((byte) 0xa6);
            command.put((byte) 0x00);
            command.put(port.getOutput());
            Ev3Brick.getDefaultBrick().sendNoReplyCommand(command);
        } else {
            ByteBuffer command = createBuffer((byte) 0xa3, (byte) 0x00, port.getOutput(), (byte) (brakeMode ? 0x01 : 0x00));
            Ev3Brick.getDefaultBrick().sendReplyCommand(command, 0, 0);
        }
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void setInverted(boolean b) {
        inverted = b;
        posInverted = b;
    }

    private ByteBuffer createBuffer(byte... bytes) {
        return ByteBuffer.wrap(bytes);
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public void setBrakeModeEnabled(boolean b) {
        brakeMode = b;
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

    @Override
    public boolean isLowerLimitSwitchActive() {
        return false;
    }

    @Override
    public boolean isUpperLimitSwitchActive() {
        return false;
    }
}
