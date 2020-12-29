// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * ADXL345 SPI Accelerometer.
 */
@SuppressWarnings({"TypeName", "PMD.UnusedPrivateField"})
public class ADXL345_SPI implements Accelerometer, Sendable, AutoCloseable {
  private static final int kPowerCtlRegister = 0x2D;
  private static final int kDataFormatRegister = 0x31;
  private static final int kDataRegister = 0x32;
  private static final double kGsPerLSB = 0.00390625;

  private static final int kAddress_Read = 0x80;
  private static final int kAddress_MultiByte = 0x40;

  private static final int kPowerCtl_Link = 0x20;
  private static final int kPowerCtl_AutoSleep = 0x10;
  private static final int kPowerCtl_Measure = 0x08;
  private static final int kPowerCtl_Sleep = 0x04;

  private static final int kDataFormat_SelfTest = 0x80;
  private static final int kDataFormat_SPI = 0x40;
  private static final int kDataFormat_IntInvert = 0x20;
  private static final int kDataFormat_FullRes = 0x08;
  private static final int kDataFormat_Justify = 0x04;

  public enum Axes {
    kX((byte) 0x00),
    kY((byte) 0x02),
    kZ((byte) 0x04);

    /**
     * The integer value representing this enumeration.
     */
    public final byte value;

    Axes(byte value) {
      this.value = value;
    }
  }

  @SuppressWarnings("MemberName")
  public static class AllAxes {
    public double XAxis;
    public double YAxis;
    public double ZAxis;
  }

  protected SPI m_spi;

  protected SimDevice m_simDevice;
  protected SimEnum m_simRange;
  protected SimDouble m_simX;
  protected SimDouble m_simY;
  protected SimDouble m_simZ;

  /**
   * Constructor.
   *
   * @param port  The SPI port that the accelerometer is connected to
   * @param range The range (+ or -) that the accelerometer will measure.
   */
  public ADXL345_SPI(SPI.Port port, Range range) {
    m_spi = new SPI(port);
    // simulation
    m_simDevice = SimDevice.create("Accel:ADXL345_SPI", port.value);
    if (m_simDevice != null) {
      m_simRange = m_simDevice.createEnumDouble("range", SimDevice.Direction.kOutput,
          new String[] {"2G", "4G", "8G", "16G"}, new double[] {2.0, 4.0, 8.0, 16.0}, 0);
      m_simX = m_simDevice.createDouble("x", SimDevice.Direction.kInput, 0.0);
      m_simY = m_simDevice.createDouble("y", SimDevice.Direction.kInput, 0.0);
      m_simZ = m_simDevice.createDouble("z", SimDevice.Direction.kInput, 0.0);
    }
    init(range);
    SendableRegistry.addLW(this, "ADXL345_SPI", port.value);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
    if (m_spi != null) {
      m_spi.close();
      m_spi = null;
    }
    if (m_simDevice != null) {
      m_simDevice.close();
      m_simDevice = null;
    }
  }

  /**
   * Set SPI bus parameters, bring device out of sleep and set format.
   *
   * @param range The range (+ or -) that the accelerometer will measure.
   */
  private void init(Range range) {
    m_spi.setClockRate(500000);
    m_spi.setMSBFirst();
    m_spi.setSampleDataOnTrailingEdge();
    m_spi.setClockActiveLow();
    m_spi.setChipSelectActiveHigh();

    // Turn on the measurements
    byte[] commands = new byte[2];
    commands[0] = kPowerCtlRegister;
    commands[1] = kPowerCtl_Measure;
    m_spi.write(commands, 2);

    setRange(range);

    HAL.report(tResourceType.kResourceType_ADXL345, tInstances.kADXL345_SPI);
  }

  @Override
  public void setRange(Range range) {
    final byte value;

    switch (range) {
      case k2G:
        value = 0;
        break;
      case k4G:
        value = 1;
        break;
      case k8G:
        value = 2;
        break;
      case k16G:
        value = 3;
        break;
      default:
        throw new IllegalArgumentException(range + " unsupported");
    }

    // Specify the data format to read
    byte[] commands = new byte[]{kDataFormatRegister, (byte) (kDataFormat_FullRes | value)};
    m_spi.write(commands, commands.length);

    if (m_simRange != null) {
      m_simRange.set(value);
    }
  }

  @Override
  public double getX() {
    return getAcceleration(Axes.kX);
  }

  @Override
  public double getY() {
    return getAcceleration(Axes.kY);
  }

  @Override
  public double getZ() {
    return getAcceleration(Axes.kZ);
  }

  /**
   * Get the acceleration of one axis in Gs.
   *
   * @param axis The axis to read from.
   * @return Acceleration of the ADXL345 in Gs.
   */
  public double getAcceleration(ADXL345_SPI.Axes axis) {
    if (axis == Axes.kX && m_simX != null) {
      return m_simX.get();
    }
    if (axis == Axes.kY && m_simY != null) {
      return m_simY.get();
    }
    if (axis == Axes.kZ && m_simZ != null) {
      return m_simZ.get();
    }
    ByteBuffer transferBuffer = ByteBuffer.allocate(3);
    transferBuffer.put(0,
        (byte) ((kAddress_Read | kAddress_MultiByte | kDataRegister) + axis.value));
    m_spi.transaction(transferBuffer, transferBuffer, 3);
    // Sensor is little endian
    transferBuffer.order(ByteOrder.LITTLE_ENDIAN);

    return transferBuffer.getShort(1) * kGsPerLSB;
  }

  /**
   * Get the acceleration of all axes in Gs.
   *
   * @return An object containing the acceleration measured on each axis of the ADXL345 in Gs.
   */
  public ADXL345_SPI.AllAxes getAccelerations() {
    ADXL345_SPI.AllAxes data = new ADXL345_SPI.AllAxes();
    if (m_simX != null && m_simY != null && m_simZ != null) {
      data.XAxis = m_simX.get();
      data.YAxis = m_simY.get();
      data.ZAxis = m_simZ.get();
      return data;
    }
    if (m_spi != null) {
      ByteBuffer dataBuffer = ByteBuffer.allocate(7);
      // Select the data address.
      dataBuffer.put(0, (byte) (kAddress_Read | kAddress_MultiByte | kDataRegister));
      m_spi.transaction(dataBuffer, dataBuffer, 7);
      // Sensor is little endian... swap bytes
      dataBuffer.order(ByteOrder.LITTLE_ENDIAN);

      data.XAxis = dataBuffer.getShort(1) * kGsPerLSB;
      data.YAxis = dataBuffer.getShort(3) * kGsPerLSB;
      data.ZAxis = dataBuffer.getShort(5) * kGsPerLSB;
    }
    return data;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("3AxisAccelerometer");
    NetworkTableEntry entryX = builder.getEntry("X");
    NetworkTableEntry entryY = builder.getEntry("Y");
    NetworkTableEntry entryZ = builder.getEntry("Z");
    builder.setUpdateTable(() -> {
      AllAxes data = getAccelerations();
      entryX.setDouble(data.XAxis);
      entryY.setDouble(data.YAxis);
      entryZ.setDouble(data.ZAxis);
    });
  }
}
