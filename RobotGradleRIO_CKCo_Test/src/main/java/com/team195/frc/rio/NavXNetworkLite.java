package com.team195.frc.rio;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * Driver for a NavX board. Basically a wrapper for the {@link AHRS} class
 */
public class NavXNetworkLite {
	protected class Callback implements ITimestampedDataSubscriber {
		@Override
		public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase update,
		                                    Object context) {
			synchronized (NavXNetworkLite.this) {
				// This handles the fact that the sensor is inverted from our coordinate conventions.
				if (mLastSensorTimestampMs != kInvalidTimestamp && mLastSensorTimestampMs < sensor_timestamp) {
					mYawRateDegreesPerSecond = 1000.0 * (-mYawDegrees - update.yaw)
							/ (double) (sensor_timestamp - mLastSensorTimestampMs);
				}
				mLastSensorTimestampMs = sensor_timestamp;
				mYawDegrees = -update.yaw;
				mFusedHeading = -update.fused_heading;
			}
		}
	}

	protected AHRS mAHRS;

	protected double mYawDegrees;
	protected double mFusedHeading;
	protected double mYawRateDegreesPerSecond;
	protected final long kInvalidTimestamp = -1;
	protected long mLastSensorTimestampMs;

	public NavXNetworkLite() {
		this(SPI.Port.kMXP);
	}

	public NavXNetworkLite(SPI.Port spi_port_id) {
		mAHRS = new AHRS(spi_port_id, (byte) 200);
		resetState();
		mAHRS.registerCallback(new Callback(), null);
	}

	public boolean isPresent() {
		return mAHRS.isConnected();
	}

	public synchronized boolean reset() {
		mAHRS.reset();
		resetState();
		return true;
	}

	public synchronized void zeroYaw() {
		mAHRS.zeroYaw();
		resetState();
	}

	private void resetState() {
		mLastSensorTimestampMs = kInvalidTimestamp;
		mYawDegrees = 0.0;
		mYawRateDegreesPerSecond = 0.0;
	}


	public synchronized double getYaw() {
		return mYawDegrees;
	}

	public double getRoll() {
		return mAHRS.getRoll();
	}

	public double getPitch() {
		return mAHRS.getPitch();
	}

	public double getYawRateDegreesPerSec() {
		return mYawRateDegreesPerSecond;
	}

	public double getFusedHeading() {
		return mFusedHeading;
	}
}
