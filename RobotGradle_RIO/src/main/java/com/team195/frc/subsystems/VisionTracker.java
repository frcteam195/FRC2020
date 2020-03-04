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
				mPeriodicIO.target_short_side = mCurrentTargetingLimelightNT.getEntry("tshort").getDouble(0);
				mPeriodicIO.target_long_side = mCurrentTargetingLimelightNT.getEntry("tlong").getDouble(0);
				mPeriodicIO.target_horizontal_side = mCurrentTargetingLimelightNT.getEntry("thor").getDouble(0);
				mPeriodicIO.target_vertical_side = mCurrentTargetingLimelightNT.getEntry("tvert").getDouble(0);
				mPeriodicIO.get_pipeline_value = mCurrentTargetingLimelightNT.getEntry("getpipe").getDouble(0);
				mPeriodicIO.camera_translation = new CameraTranslation(mCurrentTargetingLimelightNT.getEntry("camtran").getDoubleArray(mPeriodicIO.camera_translation_rotation_default_array));
				mPeriodicIO.camera_to_target_pose = new Pose2d(new Translation2d(mPeriodicIO.camera_translation.x, mPeriodicIO.camera_translation.y), Rotation2d.fromDegrees(mPeriodicIO.camera_translation.yaw));
				if (!mPeriodicIO.camera_to_target_pose.equals(mPeriodicIO.prev_camera_to_target_pose)) {
//					Pose2d targetToCamera = mPeriodicIO.cameraToTargetPose.inverse();
//					Pose2d cameraToTurret = CalConstants.kTurretToCamera.inverse();
//					Pose2d targetToTurret = targetToCamera.transformBy(cameraToTurret);
//					Pose2d turretToVehicle = CalConstants.kVehicleToTurret.inverse();
//					Pose2d targetToVehicle = targetToTurret.transformBy(turretToVehicle);
//					Pose2d fieldToVehicle = TargetingConstants.fieldToOuterTarget.transformBy(targetToVehicle);
//					RobotState.getInstance().addFieldToVehicleObservation(Timer.getFPGATimestamp(), fieldToVehicle);
					Pose2d latestAbsoluteFieldToVehicle = TargetingConstants.fieldToOuterTarget.transformBy(
							mPeriodicIO.camera_to_target_pose.inverse()    //May want to use robot angle here instead of angle from Camera pose
							.transformBy(CalConstants.kTurretToCamera.inverse())
							.transformBy(Turret.getInstance().getLatestVehicleToTurretPose().inverse())
					);
					RobotState.getInstance().addFieldToVehicleObservation(Timer.getFPGATimestamp(), latestAbsoluteFieldToVehicle);
					mPeriodicIO.prev_camera_to_target_pose = mPeriodicIO.camera_to_target_pose;
				}
			}
			else {
				mPeriodicIO.target_valid = 0;
				mPeriodicIO.target_horizontal_deviation = 0;
				mPeriodicIO.target_vertical_deviation = 0;
				mPeriodicIO.target_area = 0;
				mPeriodicIO.target_skew = 0;
				mPeriodicIO.target_latency = 0;
				mPeriodicIO.target_short_side = 0;
				mPeriodicIO.target_long_side = 0;
				mPeriodicIO.target_horizontal_side = 0;
				mPeriodicIO.target_vertical_side = 0;
				mPeriodicIO.get_pipeline_value = 0;
				mPeriodicIO.camera_translation = CameraTranslation.identity;
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

//		NetworkTableEntry ledMode = mCurrentTargetingLimelightNT.getValue().getEntry("ledMode");
//		NetworkTableEntry camMode = mCurrentTargetingLimelightNT.getValue().getEntry("camMode");
//		NetworkTableEntry stream = mCurrentTargetingLimelightNT.getValue().getEntry("stream");
//		NetworkTableEntry snapshot = mCurrentTargetingLimelightNT.getValue().getEntry("snapshot");

		mPeriodicIO.vision_loop_time += loopTimer.hasElapsed();
	}

	public TargetMode getTargetMode() {
		return mTargetMode;
	}

	public static class CameraTranslation {
		public final double x;
		public final double y;
		public final double z;
		public final double pitch;
		public final double yaw;
		public final double roll;
		public static final CameraTranslation identity = new CameraTranslation(new double[6]);
		CameraTranslation(double[] inputData) {
			if (inputData.length == 6) {
				x = inputData[0];
				y = inputData[1];
				z = inputData[2];
				pitch = inputData[3];
				yaw = inputData[4];
				roll = inputData[5];
			} else {
				x = 0;
				y = 0;
				z = 0;
				pitch = 0;
				yaw = 0;
				roll = 0;
			}
		}

		@Override
		public String toString() {
			return "(x: " + x + ", y: " + y + ", theta: " + yaw + ")";
		}
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
		double target_short_side;
		double target_long_side;
		double target_horizontal_side;
		double target_vertical_side;
		double target_distance;
		double get_pipeline_value;
		public Pose2d camera_to_target_pose;
		Pose2d prev_camera_to_target_pose;
		public CameraTranslation camera_translation;
		double[] camera_translation_rotation_default_array = new double[6];

		ArrayList<Translation2d> point_array = new ArrayList<>();

		//Written values
		int pipeline_front;
		public double vision_loop_time;
	}

	public enum TargetMode {
		AUTO_TARGET;
	}
}
