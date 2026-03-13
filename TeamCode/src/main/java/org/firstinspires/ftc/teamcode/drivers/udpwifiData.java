// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;

public class udpwifiData {
  private final ElapsedTime timestamp = new ElapsedTime();

  String RcvIP = "192.168.43.12"; // the IP to send data to
  int Port = 3000;
  int Places = 100;
  int TimeRes = 1000;

  // double timeStamp = 0.0;

  // int device = 0;
  // device is the enumerated device number timeStamp is in seconds
  public udpwifiData() {}

  // sendData takes a double and converts it to a integer with the number of places.
  // Analog data is sent as a integer, Which means that integer input data must be made double to
  // use this Send method
  public void sendData(double timeStamp, int device, double AcqData) throws Exception {
    // make AcqData effectively final
    // multiply by places to move allow 23.015 to 2301
    double finalAcqData = AcqData *= Places; // this is required for the LambDa expression
    double finalTS = timeStamp *= TimeRes;
    new Thread(
            () -> {
              try {
                // This just makes the double integer with Place decimal places
                // Prepare the data removing the data after the decimal
                int SndData = (int) finalAcqData;
                int SndTime = (int) finalTS;
                // 1. Create a socket
                DatagramSocket socket = new DatagramSocket();
                String json =
                    Integer.toString(SndTime)
                        + ","
                        + Integer.toString(device)
                        + ","
                        + Integer.toString(SndData);
                // convert to byte array
                byte[] buf = json.getBytes(StandardCharsets.UTF_8);
                // set the receiver
                InetAddress address = InetAddress.getByName(RcvIP);
                // Create the packet (data, length, destination address, port)
                DatagramPacket packet = new DatagramPacket(buf, buf.length, address, Port);
                // Send the packet
                socket.send(packet);

                socket.close();
              } catch (Exception e) {
                e.printStackTrace();
              }
            })
        .start();
  } // end send
}
