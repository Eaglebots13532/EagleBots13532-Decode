// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class wifiDataGram extends LinearOpMode {

  // add telemetry
  FtcDashboard dashboard = FtcDashboard.getInstance();
  Telemetry telemetry = dashboard.getTelemetry();

  int countme = 0;

  @Override
  public void runOpMode() {

    waitForStart();
    try {
      // receiveData();
      while (opModeIsActive()) {
        sendData(countme);
        // telemetry.update();
        sleep(1000);
        countme++;
      }
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public void sendData(int countme) throws Exception {
    new Thread(
            () -> {
              try {

                // 1. Create a socket
                DatagramSocket socket = new DatagramSocket();

                // 2. Prepare the data
                String json = "Hello from Sender! " + countme;
                // byte[] buf = message.getBytes();
                // ipString json = "{\"t\":debug,\"d\":{\"flywheel_rpm\":3200}}";
                byte[] buf = json.getBytes(StandardCharsets.UTF_8);

                InetAddress address = InetAddress.getByName("192.168.43.12");

                // 3. Create the packet (data, length, destination address, port)
                DatagramPacket packet = new DatagramPacket(buf, buf.length, address, 3000);

                // 4. Send the packet
                socket.send(packet);
                telemetry.addLine("Message sent to " + address);
                telemetry.update();

                socket.close();
              } catch (Exception e) {
                e.printStackTrace();
              }
            })
        .start();
  } // end send

  public void receiveData() throws Exception {
    // 1. Create a socket bound to a specific port
    DatagramSocket socket = new DatagramSocket(3000);

    // 2. Create a buffer to store incoming data
    byte[] buffer = new byte[1024];
    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

    telemetry.addLine("Waiting for packet...");

    // 3. Receive the packet (blocks here)
    socket.receive(packet);

    // 4. Extract data
    String received = new String(packet.getData(), 0, packet.getLength());
    telemetry.addLine("Received: " + received + " - count: " + countme);

    socket.close();
  }

  public void NoiseSignal() {
    Random rand = new Random();
    double[] noise = new double[1000];

    for (int i = 0; i < noise.length; i++) {
      // Generates values between -1.0 and 1.0
      noise[i] = (rand.nextDouble() * 2.0) - 1.0;
    }
  }
}
