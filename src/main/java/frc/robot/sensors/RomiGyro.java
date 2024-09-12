package frc.robot.sensors;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class RomiGyro {
  private final SimDevice simDevice;
  private final SimDouble simRateX;
  private final SimDouble simRateY;
  private final SimDouble simRateZ;
  private final SimDouble simAngleX;
  private final SimDouble simAngleY;
  private final SimDouble simAngleZ;

  private double angleXOffset;
  private double angleYOffset;
  private double angleZOffset;

  /** Create a new RomiGyro. */
  public RomiGyro() {
    simDevice = SimDevice.create("Gyro:RomiGyro");
    if (simDevice != null) {
      simDevice.createBoolean("init", Direction.kOutput, true);
      simRateX = simDevice.createDouble("rate_x", Direction.kInput, 0.0);
      simRateY = simDevice.createDouble("rate_y", Direction.kInput, 0.0);
      simRateZ = simDevice.createDouble("rate_z", Direction.kInput, 0.0);

      simAngleX = simDevice.createDouble("angle_x", Direction.kInput, 0.0);
      simAngleY = simDevice.createDouble("angle_y", Direction.kInput, 0.0);
      simAngleZ = simDevice.createDouble("angle_z", Direction.kInput, 0.0);
    } else {
      simRateX = null;
      simRateY = null;
      simRateZ = null;
      simAngleX = null;
      simAngleY = null;
      simAngleZ = null;
    }
  }

  /**
   * Get the rate of turn in degrees-per-second around the X-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateX() {
    if (simRateX != null) {
      return simRateX.get();
    }

    return 0.0;
  }

  /**
   * Get the rate of turn in degrees-per-second around the Y-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateY() {
    if (simRateY != null) {
      return simRateY.get();
    }

    return 0.0;
  }

  /**
   * Get the rate of turn in degrees-per-second around the Z-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateZ() {
    if (simRateZ != null) {
      return simRateZ.get();
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around X-axis in degrees
   */
  public double getAngleX() {
    if (simAngleX != null) {
      return simAngleX.get() - angleXOffset;
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around Y-axis in degrees
   */
  public double getAngleY() {
    if (simAngleY != null) {
      return simAngleY.get() - angleYOffset;
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the Z-axis.
   *
   * @return current angle around Z-axis in degrees
   */
  public double getAngleZ() {
    if (simAngleZ != null) {
      return simAngleZ.get() - angleZOffset;
    }

    return 0.0;
  }

  /** Reset the gyro angles to 0. */
  public void reset() {
    if (simAngleX != null) {
      angleXOffset = simAngleX.get();
      angleYOffset = simAngleY.get();
      angleZOffset = simAngleZ.get();
    }
  }

  public double getAngle() {
    return getAngleZ();
  }

  public double getRate() {
    return getRateZ();
  }

  /** Close out the SimDevice. */
  public void close() {
    if (simDevice != null) {
      simDevice.close();
    }
  }
}
