package com.coolioasjulio.ev3;
import java.io.*;
import javax.microedition.io.*;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import java.io.*;

public class Test {
    static final String mac_addr = "001653419a21";

    static final byte  opNop                        = (byte)  0x01;
    static final byte  DIRECT_COMMAND_REPLY         = (byte)  0x00;

    static InputStream in;
    static OutputStream out;

    public static void connectBluetooth ()
            throws IOException {
        String s = "btspp://" + mac_addr + ":1";
        StreamConnection c = (StreamConnection) Connector.open(s);
        in = c.openInputStream();
        out = c.openOutputStream();
    }

    public static ByteBuffer sendDirectCmd (ByteBuffer operations,
                                            int local_mem, int global_mem)
            throws IOException {
        ByteBuffer buffer = ByteBuffer.allocateDirect(operations.position() + 7);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.putShort((short) (operations.position() + 5));   // length
        buffer.putShort((short) 42);                            // counter
        buffer.put(DIRECT_COMMAND_REPLY);                       // type
        buffer.putShort((short) (local_mem*1024 + global_mem)); // header
        for (int i=0; i < operations.position(); i++) {         // operations
            buffer.put(operations.get(i));
        }

        byte[] cmd = new byte [buffer.position()];
        for (int i=0; i<buffer.position(); i++) cmd[i] = buffer.get(i);
        out.write(cmd);
        printHex("Sent", buffer);

        byte[] reply = new byte[global_mem + 5];
        in.read(reply);
        buffer = ByteBuffer.wrap(reply);
        buffer.position(reply.length);
        printHex("Recv", buffer);

        return buffer;
    }

    public static void printHex(String desc, ByteBuffer buffer) {
        System.out.print(desc + " 0x|");
        for (int i= 0; i < buffer.position() - 1; i++) {
            System.out.printf("%02X:", buffer.get(i));
        }
        System.out.printf("%02X|", buffer.get(buffer.position() - 1));
        System.out.println();
    }

    public static void main (String args[] ) {
        try {
            connectBluetooth();

            ByteBuffer operations = ByteBuffer.allocateDirect(1);
            operations.put(opNop);

            ByteBuffer reply = sendDirectCmd(operations, 0, 0);
        }
        catch (Exception e) {
            e.printStackTrace(System.err);
        }
    }
}
