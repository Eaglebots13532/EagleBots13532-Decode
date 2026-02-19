// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.drivers.odo;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
import static com.qualcomm.robotcore.util.TypeConversion.intToByteArray;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.stream.Stream;
import org.firstinspires.ftc.teamcode.drivers.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drivers.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.drivers.wpilib.geometry.Translation2d;

@I2cDeviceType
@DeviceProperties(
    name = "goBILDA® Pinpoint Odometry Computer",
    xmlTag = "goBILDAPinpoint",
    description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)")
public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {

  private int deviceID = 0;
  private int deviceVersion = 0;
  private int deviceStatus = 0;
  private int loopTime = 0;
  private float xPositionMm = 0;
  private float yPositionMm = 0;
  private float hOrientationRad = 0;
  private float xVelocityMmPerSec = 0;
  private float yVelocityMmPerSec = 0;
  private float hVelocityRadPerSec = 0;
  private float pitchRad = 0;
  private float rollRad = 0;

  private Register[] bulkReadScope = {
    Register.DEVICE_STATUS,
    Register.LOOP_TIME,
    Register.X_POSITION,
    Register.Y_POSITION,
    Register.H_ORIENTATION,
    Register.X_VELOCITY,
    Register.Y_VELOCITY,
    Register.H_VELOCITY
  };

  private ErrorDetectionType errorDetectionType = ErrorDetectionType.LOCAL_TEST;

  private static final float goBILDA_SWINGARM_POD =
      13.26291192f; // ticks-per-mm for the goBILDA Swingarm Pod
  private static final float goBILDA_4_BAR_POD =
      19.89436789f; // ticks-per-mm for the goBILDA 4-Bar Pod

  private final int CRC_SIZE = 1;

  // i2c address of the device
  public static final byte DEFAULT_ADDRESS = 0x31;

  public GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
    super(deviceClient, deviceClientIsOwned);

    this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
    super.registerArmingStateCallback(false);
  }

  @Override
  public Manufacturer getManufacturer() {
    return Manufacturer.Other;
  }

  @Override
  protected synchronized boolean doInitialize() {
    ((LynxI2cDeviceSynch) (deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    return true;
  }

  @Override
  public String getDeviceName() {
    return "goBILDA® Pinpoint Odometry Computer";
  }

  /**
   * Captures the length of each type of register used on the device. Aside from BULK_READ all
   * registers are 4 bytes long
   */
  private enum RegisterType {
    INT32(4),
    FLOAT(4),
    GENERIC(4),
    BULK(40);

    private final int length;

    RegisterType(int length) {
      this.length = length;
    }
  }

  /** Register map, including register address and register type */
  public enum Register {
    DEVICE_ID(1, RegisterType.INT32),
    DEVICE_VERSION(2, RegisterType.INT32),
    DEVICE_STATUS(3, RegisterType.INT32),
    DEVICE_CONTROL(4, RegisterType.INT32),
    LOOP_TIME(5, RegisterType.INT32),
    X_POSITION(8, RegisterType.FLOAT),
    Y_POSITION(9, RegisterType.FLOAT),
    H_ORIENTATION(10, RegisterType.FLOAT),
    X_VELOCITY(11, RegisterType.FLOAT),
    Y_VELOCITY(12, RegisterType.FLOAT),
    H_VELOCITY(13, RegisterType.FLOAT),
    MM_PER_TICK(14, RegisterType.FLOAT),
    X_POD_OFFSET(15, RegisterType.FLOAT),
    Y_POD_OFFSET(16, RegisterType.FLOAT),
    YAW_SCALAR(17, RegisterType.FLOAT),
    BULK_READ(18, RegisterType.BULK),
    PITCH(23, RegisterType.FLOAT),
    ROLL(24, RegisterType.FLOAT),
    SET_BULK_READ(25, RegisterType.INT32);

    private final int bVal;
    private final RegisterType registerType;

    Register(int bVal, RegisterType registerType) {
      this.bVal = bVal;
      this.registerType = registerType;
    }
  }

  /** Device Status enum that captures the current fault condition of the device */
  public enum DeviceStatus {
    NOT_READY(0),
    READY(1),
    CALIBRATING(1 << 1),
    FAULT_X_POD_NOT_DETECTED(1 << 2),
    FAULT_Y_POD_NOT_DETECTED(1 << 3),
    FAULT_NO_PODS_DETECTED(1 << 2 | 1 << 3),
    FAULT_IMU_RUNAWAY(1 << 4),
    FAULT_BAD_WRITE_CRC(1 << 7),
    FAULT_BAD_READ(1 << 8);

    private final int status;

    DeviceStatus(int status) {
      this.status = status;
    }
  }

  public enum EncoderDirection {
    FORWARD,
    REVERSED;
  }

  public enum GoBildaOdometryPods {
    goBILDA_SWINGARM_POD,
    goBILDA_4_BAR_POD;
  }

  /**
   * The kind of error correction used on the I²C communication from the device. NONE: This does not
   * check the data, and passes it directly to the user. CRC: This uses CRC8 error detection to
   * catch incorrect reads. - Only supported by devices with V3 firmware or newer. LOCAL_TEST: Is
   * "Controller only" validation that ensures that the data is !NAN, is not all zeros, and is a
   * reasonable number.
   */
  public enum ErrorDetectionType {
    NONE,
    CRC,
    LOCAL_TEST,
  }

  /**
   * Writes an int to the i2c device
   *
   * @param i the integer to write to the register
   */
  private void writeInt(int i) {
    deviceClient.write(Register.DEVICE_CONTROL.bVal, intToByteArray(i, ByteOrder.LITTLE_ENDIAN));
  }

  /**
   * Reads an int from a register of the i2c device
   *
   * @return returns an int that contains the value stored in the read register
   */
  private int readInt() {
    return byteArrayToInt(
        deviceClient.read(Register.DEVICE_VERSION.bVal, 4), ByteOrder.LITTLE_ENDIAN);
  }

  /**
   * Converts a byte array to a float value
   *
   * @param byteArray byte array to transform
   * @return the float value stored by the byte array
   */
  private float byteArrayToFloat(byte[] byteArray) {
    return ByteBuffer.wrap(byteArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
  }

  /**
   * Reads a float from a register
   *
   * @param reg the register to read
   * @return the float value stored in that register
   */
  private float readFloat(Register reg) {
    return byteArrayToFloat(deviceClient.read(reg.bVal, 4));
  }

  /**
   * Converts a float to a byte array
   *
   * @param value the float array to convert
   * @return the byte array converted from the float
   */
  private byte[] floatToByteArray(float value) {
    return ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(value).array();
  }

  /**
   * Writes a byte array to a register on the i2c device
   *
   * @param reg the register to write to
   * @param bytes the byte array to write
   */
  private void writeByteArray(Register reg, byte[] bytes) {
    deviceClient.write(reg.bVal, bytes);
  }

  /**
   * Writes a float to a register on the i2c device
   *
   * @param reg the register to write to
   * @param f the float to write
   */
  private void writeFloat(Register reg, float f) {
    byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
    deviceClient.write(reg.bVal, bytes);
  }

  /**
   * Looks up the DeviceStatus enum corresponding with an int value
   *
   * @param s int to lookup
   * @return the Odometry Computer state
   */
  private DeviceStatus lookupStatus(int s) {
    if ((s & DeviceStatus.CALIBRATING.status) != 0) {
      return DeviceStatus.CALIBRATING;
    }
    boolean xPodDetected = (s & DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
    boolean yPodDetected = (s & DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;

    if (!xPodDetected && !yPodDetected) {
      return DeviceStatus.FAULT_NO_PODS_DETECTED;
    }
    if (!xPodDetected) {
      return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
    }
    if (!yPodDetected) {
      return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
    }
    if ((s & DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0) {
      return DeviceStatus.FAULT_IMU_RUNAWAY;
    }
    if ((s & DeviceStatus.READY.status) != 0) {
      return DeviceStatus.READY;
    }
    if ((s & DeviceStatus.FAULT_BAD_READ.status) != 0) {
      return DeviceStatus.FAULT_BAD_READ;
    } else {
      return DeviceStatus.NOT_READY;
    }
  }

  /**
   * Checks to see if the register passed in is in the bulkReadScope
   *
   * @param register register to check
   * @return true if included in bulk read.
   */
  private boolean registerNotBulkRead(Register register) {
    return !Arrays.asList(bulkReadScope).contains(register);
  }

  /**
   * Saves either an int or a float to the private variable associated to a register.
   *
   * @param register to save data from
   * @param dataI the integer to write, if applicable
   * @param dataF the float to write, if applicable
   */
  private void saveData(Register register, int dataI, float dataF) {
    final int positionThreshold = 5000; // more than one FTC field in mm
    final int headingThreshold = 120; // About 20 full rotations in Radians
    final int velocityThreshold =
        10000; // 10k mm/sec is faster than an FTC robot should be going...
    final int headingVelocityThreshold = 120; // About 20 rotations per second

    switch (register) {
      case DEVICE_ID:
        deviceID = dataI;
        break;
      case DEVICE_VERSION:
        deviceVersion = dataI;
        break;
      case DEVICE_STATUS:
        deviceStatus = dataI;
        break;
      case LOOP_TIME:
        loopTime = dataI;
        break;
      case X_POSITION:
        if (errorDetectionType == ErrorDetectionType.LOCAL_TEST) {
          dataF = isPositionCorrupt(xPositionMm, dataF, positionThreshold, false);
        }
        xPositionMm = dataF;
        break;
      case Y_POSITION:
        if (errorDetectionType == ErrorDetectionType.LOCAL_TEST) {
          dataF = isPositionCorrupt(yPositionMm, dataF, positionThreshold, false);
        }
        yPositionMm = dataF;
        break;
      case H_ORIENTATION:
        if (errorDetectionType == ErrorDetectionType.LOCAL_TEST) {
          dataF = isPositionCorrupt(hOrientationRad, dataF, headingThreshold, false);
        }
        hOrientationRad = dataF;
        break;
      case X_VELOCITY:
        if (errorDetectionType == ErrorDetectionType.LOCAL_TEST) {
          dataF = isVelocityCorrupt(xVelocityMmPerSec, dataF, velocityThreshold, false);
        }
        xVelocityMmPerSec = dataF;
        break;
      case Y_VELOCITY:
        if (errorDetectionType == ErrorDetectionType.LOCAL_TEST) {
          dataF = isVelocityCorrupt(yVelocityMmPerSec, dataF, velocityThreshold, false);
        }
        yVelocityMmPerSec = dataF;
        break;
      case H_VELOCITY:
        if (errorDetectionType == ErrorDetectionType.LOCAL_TEST) {
          dataF = isVelocityCorrupt(hVelocityRadPerSec, dataF, headingVelocityThreshold, false);
        }
        hVelocityRadPerSec = dataF;
        break;
      case PITCH:
        pitchRad = dataF;
        break;
      case ROLL:
        rollRad = dataF;
        break;
    }
  }

  /**
   * Confirm that the number received is a number, and does not include a change above the threshold
   *
   * @param oldValue the reading from the previous cycle
   * @param newValue the new reading
   * @param threshold the maximum change between this reading and the previous one
   * @param bulkUpdate true if we are updating the loopTime variable. If not it should be false.
   * @return newValue if the position is good, oldValue otherwise
   */
  private Float isPositionCorrupt(
      float oldValue, float newValue, int threshold, boolean bulkUpdate) {
    boolean noData = bulkUpdate && (loopTime < 1);

    boolean isCorrupt =
        noData || Float.isNaN(newValue) || Math.abs(newValue - oldValue) > threshold;

    if (!isCorrupt) {
      return newValue;
    }

    deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
    return oldValue;
  }

  /**
   * Confirm that the number received is a number, and does not include a change above the threshold
   *
   * @param oldValue the reading from the previous cycle
   * @param newValue the new reading
   * @param threshold the velocity allowed to be reported
   * @param bulkUpdate true if we are updating the loopTime variable. If not it should be false.
   * @return newValue if the velocity is good, oldValue otherwise
   */
  private Float isVelocityCorrupt(
      float oldValue, float newValue, int threshold, boolean bulkUpdate) {
    boolean noData = bulkUpdate && (loopTime < 1);

    boolean isCorrupt = noData || Float.isNaN(newValue) || Math.abs(newValue) > threshold;

    if (!isCorrupt) {
      return newValue;
    }

    deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
    return oldValue;
  }

  /**
   * Reads the BULK_READ register depending on how the bulkReadScope is configured and saves data to
   * local variables. if CRC is enabled, and a bad CRC is detected on the BulkRead, then no data
   * will be saved.
   */
  private void flexBulkRead() {
    byte[] bArr;

    if (errorDetectionType == ErrorDetectionType.CRC) {
      bArr =
          deviceClient.read(
              Register.BULK_READ.bVal,
              (bulkReadScope.length * RegisterType.GENERIC.length) + CRC_SIZE);
      if (!checkCRC(bArr, RegisterType.BULK)) {
        return;
      }
    } else {
      bArr =
          deviceClient.read(
              Register.BULK_READ.bVal, (bulkReadScope.length * RegisterType.GENERIC.length));
    }

    for (int i = 0; i < bulkReadScope.length; i++) {
      int index = i * RegisterType.GENERIC.length;
      switch (bulkReadScope[i].registerType) {
        case INT32:
          int dataI =
              byteArrayToInt(
                  Arrays.copyOfRange(bArr, index, index + bulkReadScope[i].registerType.length),
                  ByteOrder.LITTLE_ENDIAN);
          saveData(bulkReadScope[i], dataI, 0);
          break;
        case FLOAT:
          float dataF =
              byteArrayToFloat(
                  Arrays.copyOfRange(bArr, index, index + bulkReadScope[i].registerType.length));
          saveData(bulkReadScope[i], 0, dataF);
          break;
      }
    }
  }

  /**
   * For devices with version 1 or version 2 firmware, this reads a fixed length BULK_READ register.
   * <br>
   * A warning is thrown if CRC is requested as CRC was not enabled on V1 or V2 devices.
   */
  private void fixedBulkRead() {
    final int positionThreshold = 5000; // more than one FTC field in mm
    final int headingThreshold = 120; // About 20 full rotations in Radians
    final int velocityThreshold =
        10000; // 10k mm/sec is faster than an FTC robot should be going...
    final int headingVelocityThreshold = 120; // About 20 rotations per second

    float oldPosX = xPositionMm;
    float oldPosY = yPositionMm;
    float oldPosH = hOrientationRad;
    float oldVelX = xVelocityMmPerSec;
    float oldVelY = yVelocityMmPerSec;
    float oldVelH = hVelocityRadPerSec;

    byte[] bArr = deviceClient.read(Register.BULK_READ.bVal, 40);
    deviceStatus = byteArrayToInt(Arrays.copyOfRange(bArr, 0, 4), ByteOrder.LITTLE_ENDIAN);
    loopTime = byteArrayToInt(Arrays.copyOfRange(bArr, 4, 8), ByteOrder.LITTLE_ENDIAN);
    xPositionMm = byteArrayToFloat(Arrays.copyOfRange(bArr, 16, 20));
    yPositionMm = byteArrayToFloat(Arrays.copyOfRange(bArr, 20, 24));
    hOrientationRad = byteArrayToFloat(Arrays.copyOfRange(bArr, 24, 28));
    xVelocityMmPerSec = byteArrayToFloat(Arrays.copyOfRange(bArr, 28, 32));
    yVelocityMmPerSec = byteArrayToFloat(Arrays.copyOfRange(bArr, 32, 36));
    hVelocityRadPerSec = byteArrayToFloat(Arrays.copyOfRange(bArr, 36, 40));

    switch (errorDetectionType) {
      case CRC:
        throw new RuntimeException("CRC Error Handling Not Supported by this Firmware");
      case LOCAL_TEST:
        /*
         * Check to see if any of the floats we have received from the device are NaN or are too large
         * if they are, we return the previously read value and alert the user via the DeviceStatus Enum.
         */
        xPositionMm = isPositionCorrupt(oldPosX, xPositionMm, positionThreshold, true);
        yPositionMm = isPositionCorrupt(oldPosY, yPositionMm, positionThreshold, true);
        hOrientationRad = isPositionCorrupt(oldPosH, hOrientationRad, headingThreshold, true);
        xVelocityMmPerSec = isVelocityCorrupt(oldVelX, xVelocityMmPerSec, velocityThreshold, true);
        yVelocityMmPerSec = isVelocityCorrupt(oldVelY, yVelocityMmPerSec, velocityThreshold, true);
        hVelocityRadPerSec =
            isVelocityCorrupt(oldVelH, hVelocityRadPerSec, headingVelocityThreshold, true);
        break;
    }
  }

  /**
   * Reads a register and saves the data found there to the local variable. Uses either CRC or Local
   * error detection.
   *
   * @param register register to read
   */
  public void readRegister(Register register) {
    boolean checkCRC = (errorDetectionType == ErrorDetectionType.CRC);

    byte[] temp = deviceClient.read(register.bVal, RegisterType.GENERIC.length + CRC_SIZE);

    switch (register.registerType) {
      case INT32:
        if (checkCRC(temp, RegisterType.INT32) || !checkCRC) {
          saveData(
              register,
              byteArrayToInt(
                  Arrays.copyOfRange(temp, 0, RegisterType.INT32.length), ByteOrder.LITTLE_ENDIAN),
              0);
        } else {
          deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
        }
        break;
      case FLOAT:
        if (checkCRC(temp, RegisterType.FLOAT) || !checkCRC) {
          saveData(
              register,
              0,
              byteArrayToFloat(Arrays.copyOfRange(temp, 0, RegisterType.FLOAT.length)));
        } else {
          deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
        }
        break;
      case BULK:
        update();
        break;
    }
  }

  /**
   * checks a given byteArray[] for a valid CRC data signature by comparing a calculated CRC to the
   * one received in the read.
   *
   * @param byteArray data to validate.
   * @param registerType The kind of register validated. Can be FLOAT, INT32, or BULK.
   * @return true if CRC validates the data. False otherwise.
   */
  private boolean checkCRC(byte[] byteArray, RegisterType registerType) {
    if (registerType == RegisterType.BULK) {
      int readLength = bulkReadScope.length * RegisterType.GENERIC.length;
      if (computeCRC8(Arrays.copyOfRange(byteArray, 0, readLength))
          == byteArray[(readLength + CRC_SIZE) - 1]) {
        return true;
      } else {
        deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
        return false;
      }
    }

    if (byteArray.length > RegisterType.GENERIC.length) {
      if (computeCRC8(Arrays.copyOfRange(byteArray, 0, RegisterType.GENERIC.length))
          == byteArray[(RegisterType.GENERIC.length + CRC_SIZE) - 1]) {
        return true;
      } else {
        deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
        return false;
      }
    }
    return false;
  }

  /**
   * Computes the correct CRC8 for a byteArray.
   *
   * @param byteArray data to check
   * @return byte to compare against received CRC.
   */
  private byte computeCRC8(byte[] byteArray) {
    byte crc = (byte) 0x90;

    for (byte b : byteArray) {
      crc ^= b;
      for (int i = 0; i < 8; i++) {
        if ((crc & 0x80) != 0) {
          byte crcPolynomialValue = (byte) 0x31;
          crc = (byte) ((crc << 1) ^ crcPolynomialValue);
        } else {
          crc <<= 1;
        }
      }
    }
    byte crcFinalXorValue = (byte) 0x00;
    return (byte) (crc ^ crcFinalXorValue);
  }

  /**
   * Call this once per loop to read new data from the Odometry Computer. Data will only update once
   * this is called. On devices with firmware V3 or above, the registers read by this function can
   * be changed via .setBulkReadScope.
   */
  public void update() {
    if (deviceVersion == 0) {
      deviceVersion = readInt();
    }
    if (deviceVersion == 1 || deviceVersion == 2) {
      fixedBulkRead();
    }
    if (deviceVersion >= 3) {
      flexBulkRead();
    }
  }

  /**
   * Only supported on V3 firmware and above. This configures the registers that are read in bulk
   * when .update() is called. Use this to minimize read times based on your unique application.
   *
   * @param registers An array of registers, add registers that you need data from frequently.
   */
  public void setBulkReadScope(Register... registers) {
    if (deviceVersion == 0) {
      deviceVersion = readInt();
    }
    if (deviceVersion == 1 || deviceVersion == 2) {
      throw new RuntimeException(".setBulkReadScope is not supported by this device firmware.");
    }
    if (deviceVersion >= 3) {
      bulkReadScope = registers.clone();

      Stream<Register> reg = Arrays.stream(registers).distinct();
      ArrayList<Byte> arrayList = new ArrayList<>(registers.length);

      Iterator<Register> iter = reg.iterator();
      while (iter.hasNext()) {
        arrayList.add((byte) iter.next().bVal);
      }

      byte[] arr = new byte[arrayList.size()];
      for (int i = 0; i < arrayList.size(); i++) {
        arr[i] = arrayList.get(i);
      }
      writeByteArray(Register.SET_BULK_READ, arr); // write all registers sequentially
    }
  }

  /**
   * The kind of error correction used on the I²C communication from the device. <br>
   * <br>
   * NONE: This does not check the data, and passes it directly to the user. <br>
   * CRC: This uses CRC8 error detection to catch incorrect reads. - Only supported by devices with
   * V3 firmware or newer.<br>
   * LOCAL_TEST: "Controller only" validation that ensures that the data is !NAN, is not all zeros,
   * and is a reasonable number. This is faster than CRC but may not catch every erroneous read.<br>
   */
  public void setErrorDetectionType(ErrorDetectionType e) {
    errorDetectionType = e;
  }

  /**
   * Sets the odometry pod positions relative to the point that the odometry computer tracks around.
   * <br>
   * <br>
   * The most common tracking position is the center of the robot. <br>
   * <br>
   * The X pod offset refers to how far sideways from the tracking point the X (forward) odometry
   * pod is. Left of the center is a positive number, right of center is a negative number. <br>
   * the Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod
   * is. forward of center is a positive number, backwards is a negative number.<br>
   *
   * @param xOffsetMeters how sideways from the center of the robot is the X (forward) pod? Left+.
   * @param yOffsetMeters how far forward from the center of the robot is the Y (Strafe) pod?
   *     Forwards+.
   */
  public void setOffsets(double xOffsetMeters, double yOffsetMeters) {
    writeFloat(Register.X_POD_OFFSET, (float) (xOffsetMeters / 1000));
    writeFloat(Register.Y_POD_OFFSET, (float) (yOffsetMeters / 1000));
  }

  /**
   * Recalibrates the Odometry Computer's internal IMU. <br>
   * <br>
   * <strong> Robot MUST be stationary </strong> <br>
   * <br>
   * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes
   * approximately 0.25 seconds.
   */
  public void recalibrateIMU() {
    writeInt(1);
  }

  /**
   * Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU.
   * <br>
   * <br>
   * <strong> Robot MUST be stationary </strong> <br>
   * <br>
   * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes
   * approximately 0.25 seconds.
   */
  public void resetPosAndIMU() {
    writeInt(1 << 1);
  }

  /**
   * Can reverse the direction of each encoder.
   *
   * @param xEncoder FORWARD or REVERSED, X (forward) pod should increase when the robot is moving
   *     forward
   * @param yEncoder FORWARD or REVERSED, Y (strafe) pod should increase when the robot is moving
   *     left
   */
  public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
    if (xEncoder == EncoderDirection.FORWARD) {
      writeInt(1 << 5);
    }
    if (xEncoder == EncoderDirection.REVERSED) {
      writeInt(1 << 4);
    }

    if (yEncoder == EncoderDirection.FORWARD) {
      writeInt(1 << 3);
    }
    if (yEncoder == EncoderDirection.REVERSED) {
      writeInt(1 << 2);
    }
  }

  /**
   * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for easy access.
   * <br>
   * <br>
   *
   * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
   */
  public void setEncoderResolution(GoBildaOdometryPods pods) {
    if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
      writeByteArray(Register.MM_PER_TICK, (floatToByteArray(goBILDA_SWINGARM_POD)));
    }
    if (pods == GoBildaOdometryPods.goBILDA_4_BAR_POD) {
      writeByteArray(Register.MM_PER_TICK, (floatToByteArray(goBILDA_4_BAR_POD)));
    }
  }

  /**
   * Sets the encoder resolution in ticks per distance of the odometry pods. <br>
   * You can find this number by dividing the counts-per-revolution of your encoder by the
   * circumference of the wheel.
   *
   * @param ticksPerMm should be somewhere between 10 ticks/mm and 100 ticks/mm. A goBILDA Swingarm
   *     pod is ~13.26291192
   */
  public void setEncoderResolution(double ticksPerMm) {
    writeByteArray(Register.MM_PER_TICK, (floatToByteArray((float) ticksPerMm)));
  }

  /**
   * Tuning this value should be unnecessary.<br>
   * The goBILDA Odometry Computer has a per-device tuned yaw offset already applied when you
   * receive it.<br>
   * <br>
   * This is a scalar that is applied to the gyro's yaw value. Increasing it will mean it will
   * report more than one degree for every degree the sensor fusion algorithm measures. <br>
   * <br>
   * You can tune this variable by rotating the robot a large amount (10 full turns is a good
   * starting place) and comparing the amount that the robot rotated to the amount measured.
   * Rotating the robot exactly 10 times should measure 3600°. If it measures more or less, divide
   * moved amount by the measured amount and apply that value to the Yaw Offset.<br>
   * <br>
   * If you find that to get an accurate heading number you need to apply a scalar of more than
   * 1.05, or less than 0.95, your device may be bad. Please reach out to tech@gobilda.com
   *
   * @param yawScalar A scalar for the robot's heading.
   */
  public void setYawScalar(double yawScalar) {
    writeByteArray(Register.YAW_SCALAR, (floatToByteArray((float) yawScalar)));
  }

  /**
   * Checks the deviceID of the Odometry Computer. Should return 2.
   *
   * @return 2 if device is functional.
   */
  public int getDeviceID() {
    if (registerNotBulkRead(Register.DEVICE_ID)) {
      readRegister(Register.DEVICE_ID);
    }
    return deviceID;
  }

  /**
   * @return the firmware version of the Odometry Computer
   */
  public int getDeviceVersion() {
    if (registerNotBulkRead(Register.DEVICE_VERSION)) {
      readRegister(Register.DEVICE_VERSION);
    }
    if (deviceVersion == 1 || deviceVersion == 2) {
      return 1;
    } else if (deviceVersion == 3) {
      return 2;
    }
    return 0;
  }

  /**
   * Device Status stores any faults the Odometry Computer may be experiencing. These faults
   * include:
   *
   * @return one of the following states:<br>
   *     NOT_READY - The device is currently powering up. And has not initialized yet. RED LED<br>
   *     READY - The device is currently functioning as normal. GREEN LED<br>
   *     CALIBRATING - The device is currently recalibrating the gyro. RED LED<br>
   *     FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in. PURPLE LED <br>
   *     FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in. BLUE LED <br>
   *     FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in. ORANGE LED <br>
   *     FAULT_BAD_READ - Aa bad I²C read has been detected, the result reported is a duplicate of
   *     the last good read.
   */
  public DeviceStatus getDeviceStatus() {
    if (registerNotBulkRead(Register.DEVICE_STATUS)) {
      readRegister(Register.DEVICE_STATUS);
    }
    return lookupStatus(deviceStatus);
  }

  /**
   * Checks the Odometry Computer's most recent loop time.<br>
   * <br>
   * If values less than 500, or more than 1100 are commonly seen here, there may be something wrong
   * with your device. Please reach out to tech@gobilda.com
   *
   * @return loop time in microseconds (1/1,000,000 seconds)
   */
  public int getLoopTime() {
    if (registerNotBulkRead(Register.LOOP_TIME)) {
      readRegister(Register.LOOP_TIME);
    }
    return loopTime;
  }

  /**
   * Checks the Odometry Computer's most recent loop frequency.<br>
   * <br>
   * If values less than 900, or more than 2000 are commonly seen here, there may be something wrong
   * with your device. Please reach out to tech@gobilda.com
   *
   * @return Pinpoint Frequency in Hz (loops per second),
   */
  public double getFrequency() {
    if (loopTime != 0) {
      return 1000000.0 / loopTime;
    } else {
      return 0;
    }
  }

  /**
   * Send a position that the Pinpoint should use to track your robot relative to. You can use this
   * to update the estimated position of your robot with new external sensor data, or to run a robot
   * in field coordinates.
   *
   * @param pose The new robot pose.
   */
  public void setPose(Pose2d pose) {
    setTranslation(pose.getTranslation());
    setHeading(pose.getRotation());
  }

  /**
   * Send a position that the Pinpoint should use to track your robot relative to. You can use this
   * to update the estimated position of your robot with new external sensor data, or to run a robot
   * in field coordinates.
   *
   * @param translation The new robot translation.
   */
  public void setTranslation(Translation2d translation) {
    writeByteArray(Register.X_POSITION, (floatToByteArray((float) (translation.getX() / 1000))));
    writeByteArray(Register.Y_POSITION, (floatToByteArray((float) (translation.getY() / 1000))));
  }

  /**
   * Send a heading that the Pinpoint should use to track your robot relative to. You can use this
   * to update the estimated position of your robot with new external sensor data, or to run a robot
   * in field coordinates.
   *
   * @param heading The new heading you'd like the Pinpoint to track your robot relive to.
   */
  public void setHeading(Rotation2d heading) {
    writeByteArray(Register.H_ORIENTATION, (floatToByteArray((float) heading.getRadians())));
  }

  /**
   * @return the estimated X (forward) position of the robot
   */
  public double getXPosMeters() {
    return xPositionMm / 1000;
  }

  /**
   * @return the estimated Y (Strafe) position of the robot.
   */
  public double getYPosMeters() {
    return yPositionMm / 1000;
  }

  /**
   * @return the normalized estimated yaw position of the robot.
   */
  public Rotation2d getYaw() {
    return new Rotation2d(hOrientationRad);
  }

  /**
   * @return The estimated pose of the robot.
   */
  public Pose2d getPose() {
    return new Pose2d(getXPosMeters(), getYPosMeters(), getYaw());
  }

  /**
   * @return the estimated X (forward) velocity of the robot in specified unit/sec
   */
  public double getVelXMetersPerSecond() {
    return xVelocityMmPerSec / 1000;
  }

  /**
   * @return the estimated Y (strafe) velocity of the robot in specified unit/sec
   */
  public double getVelYMetersPerSec() {
    return yVelocityMmPerSec / 1000;
  }

  /**
   * @return the estimated H (heading) velocity of the robot in specified unit/sec
   */
  public double getYawVelocityRadPerSec() {
    return hVelocityRadPerSec;
  }

  /**
   * @return the current pitch of the device.
   */
  public Rotation2d getPitch() {
    if (deviceVersion == 0) {
      readInt();
    }
    if (deviceVersion == 1 || deviceVersion == 2) {
      throw new RuntimeException("IMU Pitch output is not supported on this device firmware");
    } else {
      if (registerNotBulkRead(Register.PITCH)) {
        pitchRad = readFloat(Register.PITCH);
      }
      return new Rotation2d(pitchRad);
    }
  }

  /**
   * @return the current roll of the device.
   */
  public Rotation2d getRoll() {
    if (deviceVersion == 0) {
      readInt();
    }
    if (deviceVersion == 1 || deviceVersion == 2) {
      throw new RuntimeException("IMU Roll output is not supported on this device firmware");
    } else {
      if (registerNotBulkRead(Register.ROLL)) {
        rollRad = readFloat(Register.ROLL);
      }
      return new Rotation2d(rollRad);
    }
  }
}
