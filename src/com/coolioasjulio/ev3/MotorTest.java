package com.coolioasjulio.ev3;

import javax.microedition.io.Connector;
import javax.microedition.io.StreamConnection;
import java.io.DataOutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class MotorTest {
    public static void main(String[] args) throws Exception {
        String s = "btspp://001653419a21:1";
        StreamConnection c = (StreamConnection) Connector.open(s);
        DataOutputStream out = c.openDataOutputStream();
        System.out.println("Connected!");
        ByteBuffer buffer = ByteBuffer.allocateDirect(15);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
//        buffer.putShort((short) 0xD);
//        buffer.putShort((short) 0x2A);
//        buffer.put((byte) 0x80);
//        buffer.putShort((short) 0);
//        buffer.put((byte) 0xa4);
//        buffer.put((byte) 0x00);
//        buffer.put((byte) 0x0f);
//        buffer.putShort((short) 0x6481);
//        buffer.put((byte) 0xa6);
//        buffer.put((byte) 0x00);
//        buffer.put((byte) 0x0f);
        buffer.putShort((short) 0x9);
        buffer.putShort((short) 0x2A);
        buffer.put((byte) 0x00);
        buffer.putShort((short) 0);
        buffer.put((byte) 0xa3);
        buffer.put((byte) 0x00);
        buffer.put((byte) 0x0f);
        buffer.put((byte) 0);

        byte[] arr = new byte[buffer.position()];
        buffer.rewind();
        buffer.get(arr);
        for(byte b : arr) {
            System.out.println(Integer.toHexString(b));
            out.write(b);
        }
        out.flush();
        System.out.println("printed!");
        Thread.sleep(5000);
    }
}
