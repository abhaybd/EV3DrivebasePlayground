package com.coolioasjulio.ev3;

import javax.microedition.io.Connector;
import javax.microedition.io.StreamConnection;
import java.io.Closeable;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class Ev3Brick implements Closeable {
    private static Ev3Brick defaultBrick;

    public static synchronized Ev3Brick getDefaultBrick() {
        if (defaultBrick == null) {
            defaultBrick = new Ev3Brick();
        }
        return defaultBrick;
    }

    private StreamConnection c;
    private DataInputStream in;
    private OutputStream out;

    public Ev3Brick() {
        try {
            c = (StreamConnection) Connector.open("btspp://001653419a21:1");
            in = c.openDataInputStream();
            out = c.openOutputStream();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public synchronized void sendCommand(ByteBuffer command, boolean reply, int localMem, int globalMem) {
        byte[] arr = new byte[command.limit()];
        command.rewind().get(arr);
        ByteBuffer buffer = ByteBuffer.wrap(new byte[arr.length + 7]);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.putShort((short) (arr.length + 5));
        buffer.putShort((short) 0x2a); // unused, I think
        buffer.put((byte) (reply ? 0x00 : 0x80)); // type: no reply or reply
        buffer.putShort((short) ((localMem << 10) + (globalMem & 0xfffff))); // memory
        buffer.put(arr);
        try {
            out.write(buffer.array());
            out.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public synchronized void sendNoReplyCommand(ByteBuffer command) {
        sendCommand(command, false, 0, 0);
    }

    public synchronized byte[] sendReplyCommand(ByteBuffer command, int localMem, int globalMem) {
        sendCommand(command, true, localMem, globalMem);
        try {
            int len = Short.reverseBytes(in.readShort());
            in.readShort(); // drop this one
            int status = in.read();
            if (status != 0x02) {
                //throw new IllegalStateException("Not ok!");
            }
            if (len < 0) {
                System.err.println("Weird len: " + len + " for command: " + command);
            }
            byte[] arr = new byte[len - 3];
            in.readFully(arr);
            //System.out.printf("Received response: %s\n", Arrays.toString(arr));
            return arr;
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void close() throws IOException {
        c.close();
    }
}
