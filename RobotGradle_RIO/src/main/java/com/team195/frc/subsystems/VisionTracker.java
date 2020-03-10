package com.team195.frc.subsystems;

import com.team195.frc.RobotState;
import com.team195.frc.constants.CalConstants;
import com.team195.frc.constants.TargetingConstants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.ReflectingLogDataGenerator;
import com.team195.lib.util.ElapsedTimer;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public class VisionTracker extends Subsystem {
	private static VisionTracker mInstance = new VisionTracker();
	private PeriodicIO mPeriodicIO = new PeriodicIO();
	private ReflectingLogDataGenerator<PeriodicIO> mLogDataGenerator = new ReflectingLogDataGenerator<>(PeriodicIO.class);

	private TargetMode mTargetMode = TargetMode.AUTO_TARGET;
	private boolean mVisionEnabled = false;

	private NetworkTable mCurrentTargetingLimelightNT;

	private NetworkTable limelightTurret = NetworkTableInstance.getDefault().getTable("limelight-turret");
	private NetworkTableEntry pipelineEntry = limelightTurret.getEntry("pipeline");

	private final ElapsedTimer loopTimer = new ElapsedTimer();

	public static VisionTracker getInstance() {
		return mInstance;
	}

	private VisionTracker() {

	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (VisionTracker.this) {

			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (VisionTracker.this) {
			}
		}

		@Override
		public void onLoop(double timestamp) {
			loopTimer.start();
			synchronized (VisionTracker.this) {
				switch (mTargetMode) {
					case AUTO_TARGET:
						mPeriodicIO.pipeline_front = mVisionEnabled ? 1 : 0;
						mCurrentTargetingLimelightNT = limelightTurret;
						break;
					default:
						mPeriodicIO.pipeline_front = mVisionEnabled ? 1 : 0;
						break;
				}
			}
			mPeriodicIO.vision_loop_time += loopTimer.hasElapsed();
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}

		@Override
		public String getName() {
			return "VisionTracker";
		}
	};

	@Override
	public void registerEnabledLoops(ILooper in) {
		in.register(mLoop);
	}

	@Override
	public void stop() {
		setVisionEnabled(false);
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		return false;
	}

	@Override
	public boolean runDiagnostics() {
		return true;
	}

	public boolean isVisionEnabled() {
		return mVisionEnabled;
	}

	public boolean isTargetFound() {
		return mVisionEnabled && mPeriodicIO.target_valid > 0;
	}

	public double getTargetDistance() {
		return mVisionEnabled ? mPeriodicIO.target_distance : 0;
	}

	public double getTargetHorizAngleDev() {
		return mVisionEnabled ? mPeriodicIO.target_horizontal_deviation : 0;
	}

	public double getTargetVertAngleDev() {
		return mVisionEnabled ? mPeriodicIO.target_vertical_deviation : 0;
	}

	public synchronized void setVisionEnabled(boolean enabled) {
		mVisionEnabled = enabled;
	}

	public synchronized void setTargetMode(TargetMode targetMode) {
		mTargetMode = targetMode;
	}

	public double getTargetSkew() {
		return mPeriodicIO.target_skew;
	}

	public Pose2d getCameraToTargetPose() {
		return mPeriodicIO.camera_to_target_pose;
	}

	public Translation2d getCameraToTargetTranslation() {
		// Convert to spherical coordinates https://en.wikipedia.org/wiki/Spherical_coordinate_system
		Rotation2d phi = Rotation2d.fromDegrees(-1 * mPeriodicIO.target_horizontal_deviation);
		Rotation2d theta = Rotation2d.fromDegrees(90).rotateBy(Rotation2d.fromDegrees(mPeriodicIO.target_vertical_deviation + CalConstants.kCameraLensAngleToHorizontal).inverse());

		// Convert to cartesian unit vector (radius r is implicitly 1, inclination theta, azimuth phi)
		double vector_x = theta.sin() * phi.cos();
		double vector_y = theta.sin() * phi.sin();
		double vector_z = theta.cos();

		// Determine the scaling of the z component of the unit vector to get to the plane
		double scaling = CalConstants.kCameraLensHeightToTargetHeightDelta / vector_z;

		// Scale the x and y component by said scaling.
		return new Translation2d(vector_x * scaling, vector_y * scaling);
	}

	@Override
	public synchronized List<Object> generateReport() {
		loopTimer.start();
		mTmpHandle = mLogDataGenerator.generateData(mPeriodicIO);
		mPeriodicIO.vision_loop_time += loopTimer.hasElapsed();
		return mTmpHandle;
	}
	private List<Object> mTmpHandle;

	@Override
	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		try {
			if (mVisionEnabled) {
				mPeriodicIO.target_valid = mCurrentTargetingLimelightNT.getEntry("tv").getDouble(0);
				mPeriodicIO.target_horizontal_deviation = mCurrentTargetingLimelightNT.getEntry("tx").getDouble(0);
				mPeriodicIO.target_vertical_deviation = mCurrentTargetingLimelightNT.getEntry("ty").getDouble(0);
				mPeriodicIO.target_area = mCurrentTargetingLimelightNT.getEntry("ta").getDouble(0);
				mPeriodicIO.target_skew = mCurrentTargetingLimelightNT.getEntry("ts").getDouble(0);
				mPeriodicIO.target_latency = mCurrentTargetingLimelightNT.getEntry("tl").getDouble(0);

				if (mPeriodicIO.target_valid > 0) {
					Translation2d cameraToTargetTranslation = getCameraToTargetTranslation();
					mPeriodicIO.camera_to_target_pose = TargetingConstants.fieldToOuterTarget.transformBy(
							new Pose2d(cameraToTargetTranslation.translateBy(CalConstants.kTurretToCamera.inverse().getTranslation()), Turret.getInstance().getLatestFieldToTurretPose().getRotation()).inverse()
									.transformBy(Turret.getInstance().getLatestVehicleToTurretPose().inverse())
					);
					RobotState.getInstance().addFieldToVehicleObservation(Timer.getFPGATimestamp(), mPeriodicIO.camera_to_target_pose);
					mPeriodicIO.target_distance = cameraToTargetTranslation.distance(Translation2d.identity());
				}
			}
			else {
				mPeriodicIO.target_valid = 0;
				mPeriodicIO.target_horizontal_deviation = 0;
				mPeriodicIO.target_vertical_deviation = 0;
				mPeriodicIO.target_area = 0;
				mPeriodicIO.target_skew = 0;
				mPeriodicIO.target_latency = 0;
				mPeriodicIO.camera_to_target_pose = Pose2d.identity();
				mPeriodicIO.target_distance = 0;
			}
		}
		catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
		mPeriodicIO.vision_loop_time = loopTimer.hasElapsed();
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		loopTimer.start();
		try {
			pipelineEntry.setNumber(mPeriodicIO.pipeline_front);
		}
		catch (Exception ex) {
			ConsoleReporter.report(ex);
		}

		mPeriodicIO.vision_loop_time += loopTimer.hasElapsed();
	}

	@SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		//Read values
		public double target_valid;
		public double target_horizontal_deviation;
		double target_vertical_deviation;
		public double target_area;
		double target_skew;
		double target_latency;
		double target_distance;
		public Pose2d camera_to_target_pose;

		//Written values
		int pipeline_front;
		public double vision_loop_time;
	}

	public enum TargetMode {
		AUTO_TARGET;
	}
}
