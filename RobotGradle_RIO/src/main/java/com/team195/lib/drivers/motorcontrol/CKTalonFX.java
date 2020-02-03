package com.team195.lib.drivers.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team195.frc.constants.Constants;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.DiagnosticMessage;
import com.team195.frc.reporters.MessageLevel;
import com.team195.lib.util.CachedValue;
import com.team195.lib.util.ThreadRateControl;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Function;

public class CKTalonFX implements TuneableMotorController {
	//We can't extend TalonSRX due to some JNI object packing issue where following does not work
	//it seems getBaseID() is broken on extension
	private final TalonFX mTalonFX;
	private final ThreadRateControl trc = new ThreadRateControl();
	private int currentSelectedSlot = 0;
	private ArrayList<FeedbackConfiguration> mFeedbackConfig = new ArrayList<>();
	private double prevOutput = Double.MIN_VALUE;
	private final PDPBreaker motorBreaker;
	private AtomicBoolean prevForwardLimitVal = new AtomicBoolean(false);
	private AtomicBoolean prevReverseLimitVal = new AtomicBoolean(false);



	private CachedValue<Double> localQuadPosition;

	private CachedValue<Boolean> forwardLimitCachedValue;
	private CachedValue<Boolean> reverseLimitCachedValue;

	private boolean sensorInverted = false;

	private static final Configuration fastMasterConfig = new Configuration(5, 5, 20);
	private static final Configuration normalMasterConfig = new Configuration(10, 10, 20);
	private static final Configuration normalSlaveConfig = new Configuration(10, 100, 100);


	private double prevMotionVelocitySetpoint = 0;
	private double minSetpointOutput = 0;
	private double allowedClosedLoopError = 0;
	private MCControlMode currentControlMode = MCControlMode.Disabled;  //Force an update

	private Thread motionVoodooArbFFControlThread;
	private double motionVoodooArbFFDemand = 0;
	public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> motionVoodooArbFFLookup = new InterpolatingTreeMap<>();
	public double absoluteEncoderOffset = 0;

	private double speedNew = 0;

	private CKTalonFX(int deviceId, PDPBreaker breakerCurrent, Configuration deviceConfig) {
		initializeFeedbackList();
		mTalonFX = new TalonFX(deviceId);
		mTalonFX.configFactoryDefault();
		motorBreaker = breakerCurrent;
		doDefaultConfig(deviceConfig);
		setBrakeCoastMode(MCNeutralMode.Brake);
		initCachedValues();
	}

	public CKTalonFX(int deviceId, boolean fastMaster, PDPBreaker breakerCurrent) {
		this(deviceId, breakerCurrent, fastMaster ? fastMasterConfig : normalMasterConfig);
		set(MCControlMode.PercentOut, 0, 0, 0);
	}

	public CKTalonFX(int deviceId, CKTalonFX masterTalon, PDPBreaker breakerCurrent, boolean inverted) {
		this(deviceId, breakerCurrent, normalSlaveConfig);
		mTalonFX.follow(masterTalon.mTalonFX);
		mTalonFX.setInverted(inverted);
	}

	private void initCachedValues() {
		localQuadPosition = new CachedValue<>(100, (t) -> convertNativeUnitsToRotations(mTalonFX.getSensorCollection().getIntegratedSensorPosition() * (sensorInverted ? -1 : 1)));
		forwardLimitCachedValue = new CachedValue<>(150, (t) -> mTalonFX.getSensorCollection().isFwdLimitSwitchClosed() == 1);
		reverseLimitCachedValue = new CachedValue<>(150, (t) -> mTalonFX.getSensorCollection().isRevLimitSwitchClosed() == 1);
	}

	public TalonFXSensorCollection getSensorCollection() {
		return mTalonFX.getSensorCollection();
	}

	public void setLocalQuadPosition(double position) {
		runTalonFunctionWithRetry((t) -> mTalonFX.getSensorCollection().setIntegratedSensorPosition(convertRotationsToNativeUnits(position), Constants.kCANTimeoutMs));
	}

	private void doDefaultConfig(Configuration config) {
		runTalonFunctionWithRetry((t) -> mTalonFX.clearStickyFaults(Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS));
		runTalonFunctionWithRetry((t) -> mTalonFX.setStatusFramePeriod(StatusFrame.Status_1_General, config.STATUS_FRAME_GENERAL_1_MS, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.STATUS_FRAME_FEEDBACK0_2_MS, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs));
		configCurrentLimit(motorBreaker.value - 10, motorBreaker.value, getMSDurationForBreakerLimit(motorBreaker.value * 2, motorBreaker.value, 8));
		runTalonFunctionWithRetry((t) -> mTalonFX.configVoltageCompSaturation(12));
		runTalonFunctionWithRetry((t) -> {
			mTalonFX.enableVoltageCompensation(true);
			return mTalonFX.getLastError();
		});
		runTalonFunctionWithRetry((t) -> mTalonFX.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs));
	}

	private void initializeFeedbackList() {
		for (int i = 0; i < 4; i++) {
			mFeedbackConfig.add(new FeedbackConfiguration());
		}
	}

	//TODO: Fix Current Limiting for new API
	public void configCurrentLimit(int continuousCurrentValueA, int thresholdCurrentA, int durationOverThresholdToEnableLimitingMs) {
//		runTalonFunctionWithRetry((t) -> mTalonFX.configContinuousCurrentLimit(continuousCurrentValueA, Constants.kLongCANTimeoutMs));
//		runTalonFunctionWithRetry((t) -> mTalonFX.configPeakCurrentLimit(thresholdCurrentA, Constants.kLongCANTimeoutMs));
//		runTalonFunctionWithRetry((t) -> mTalonFX.configPeakCurrentDuration(durationOverThresholdToEnableLimitingMs, Constants.kLongCANTimeoutMs));
//		runTalonFunctionWithRetry((t) -> {
//			mTalonFX.enableCurrentLimit(true);
//			return mTalonFX.getLastError();
//		});
	}

	public void disableCurrentLimit() {
//		runTalonFunctionWithRetry((t) -> {
//			mTalonFX.enableCurrentLimit(false);
//			return mTalonFX.getLastError();
//		});
	}

	public void setFeedbackDevice(FeedbackDevice feedbackDevice) {
		mFeedbackConfig.get(currentSelectedSlot).setFeedbackDevice(feedbackDevice);
		mFeedbackConfig.get(currentSelectedSlot).setRemoteDeviceId(-1);
		runTalonFunctionWithRetry((t) -> mTalonFX.configSelectedFeedbackSensor(feedbackDevice, 0, Constants.kCANTimeoutMs));
	}

	public void setFeedbackDevice(RemoteFeedbackDevice feedbackDevice, int remoteDeviceId) {
		mFeedbackConfig.get(currentSelectedSlot).setFeedbackDevice(feedbackDevice.getFeedbackDevice());
		mFeedbackConfig.get(currentSelectedSlot).setRemoteDeviceId(remoteDeviceId);
		runTalonFunctionWithRetry((t) -> mTalonFX.configRemoteFeedbackFilter(remoteDeviceId, RemoteSensorSource.TalonSRX_SelectedSensor, feedbackDevice == RemoteFeedbackDevice.RemoteSensor1 ? 1 : 0, Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configSelectedFeedbackSensor(feedbackDevice, 0, Constants.kCANTimeoutMs));
	}

	public void configForwardLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configForwardLimitSwitchSource(type, normalOpenOrClose, Constants.kCANTimeoutMs));
	}

	public void configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID, Constants.kCANTimeoutMs));
	}


	public void configReverseLimitSwitchSource(LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configReverseLimitSwitchSource(type, normalOpenOrClose, Constants.kCANTimeoutMs));
	}

	public void configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose, int deviceID) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configReverseLimitSwitchSource(type, normalOpenOrClose, deviceID, Constants.kCANTimeoutMs));
	}

	public void configZeroOnLimit() {
		runTalonFunctionWithRetry((t) -> mTalonFX.configClearPositionOnLimitR(true, Constants.kCANTimeoutMs));
	}

	public void setSensorPhase(boolean inverted) {
		sensorInverted = inverted;
		runTalonFunctionWithRetry((t) -> {
			mTalonFX.setSensorPhase(inverted);
			return mTalonFX.getLastError();
		});
	}

	public void setInverted(boolean inverted) {
		runTalonFunctionWithRetry((t) -> {
			mTalonFX.setInverted(inverted);
			return mTalonFX.getLastError();
		});
	}

	public void setInverted(InvertType invertType) {
		runTalonFunctionWithRetry((t) -> {
			mTalonFX.setInverted(invertType);
			return mTalonFX.getLastError();
		});
	}

	public void configClosedloopRamp(double secondsFromNeutralToFull) {
		mFeedbackConfig.get(currentSelectedSlot).setClosedLoopRampRate(secondsFromNeutralToFull);
		runTalonFunctionWithRetry((t) -> mTalonFX.configClosedloopRamp(secondsFromNeutralToFull, Constants.kCANTimeoutMs));
	}

	public void configForwardSoftLimitThreshold(double rotations) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configForwardSoftLimitThreshold(convertRotationsToNativeUnits(rotations), Constants.kCANTimeoutMs));
	}

	public void configForwardSoftLimitEnable(boolean enabled) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configForwardSoftLimitEnable(enabled, Constants.kCANTimeoutMs));
	}

	public void configReverseSoftLimitThreshold(double rotations) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configReverseSoftLimitThreshold(convertRotationsToNativeUnits(rotations), Constants.kCANTimeoutMs));
	}

	public void configReverseSoftLimitEnable(boolean enabled) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configReverseSoftLimitEnable(enabled, Constants.kCANTimeoutMs));
	}

	public double getLocalQuadPosition() {
		if (mFeedbackConfig.get(currentSelectedSlot).feedbackDevice == FeedbackDevice.CTRE_MagEncoder_Relative) {
			return getPosition();
		}
		else {
			return localQuadPosition.getValue();
		}
	}

	public synchronized void setAbsoluteEncoderOffset(double offset) {
		absoluteEncoderOffset = offset;
	}

	/**
	 * Make sure you call this method before first use of set() to ensure CL ramp rate and PID gains are selected properly when using CKTalonSRX
	 * @param slotIdx Gain profile slot
	 * @param pidIdx PID ID, 0 for main, 1 for aux
	 */
	public void selectProfileSlot(int slotIdx, int pidIdx) {
		mTalonFX.selectProfileSlot(slotIdx, pidIdx);
		setCurrentSlotValue(slotIdx);
		if (currentSelectedSlot < mFeedbackConfig.size()) {
			configClosedloopRamp(mFeedbackConfig.get(currentSelectedSlot).closedLoopRampRate);
			setMotionParameters(mFeedbackConfig.get(currentSelectedSlot).motionMagicVel, mFeedbackConfig.get(currentSelectedSlot).motionMagicAccel, mFeedbackConfig.get(currentSelectedSlot).motionMagicSCurveStrength);
			FeedbackDevice f = mFeedbackConfig.get(currentSelectedSlot).feedbackDevice;
			switch (f) {
				case RemoteSensor0:
				case RemoteSensor1:
					setFeedbackDevice(RemoteFeedbackDevice.valueOf(f.value), mFeedbackConfig.get(currentSelectedSlot).remoteDeviceId);
					break;
				default:
					setFeedbackDevice(f);
					break;
			}
//			trc.start(true);
//			trc.doRateControl(100);
		}
	}

	private synchronized void setCurrentSlotValue(int slotIdx) {
		currentSelectedSlot = slotIdx;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("General Status Frame 1: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 2: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 3: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 4: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 6: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 7: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 8: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 9: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 10: " + mTalonFX.getStatusFramePeriod(StatusFrame.Status_10_Targets, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 11: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 12: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 13: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 14: " + mTalonFX.getStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, Constants.kLongCANTimeoutMs) + "\r\n");
		return sb.toString();
	}

	@Deprecated
	public void set(ControlMode mode, double outputValue) {
		set(MCControlMode.valueOf(mode), outputValue, currentSelectedSlot, 0);
	}

	@Deprecated
	public void set(ControlMode mode, double demand0, double demand1) {
		set(MCControlMode.valueOf(mode), demand0, currentSelectedSlot, demand1);
	}

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);

		if (demand + arbitraryFeedForward != prevOutput || currentSelectedSlot != slotIdx || controlMode != currentControlMode) {
			currentControlMode = controlMode;

			if (controlMode == MCControlMode.MotionVoodooArbFF) {
				motionVoodooArbFFDemand = demand;
				startMotionVoodooArbFFControlThread();
			} else {
				demand = convertDemandToNativeUnits(controlMode, demand);
				mTalonFX.set(controlMode.CTRE(), demand, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
				prevOutput = demand + arbitraryFeedForward;
			}
		}
	}

	private void startMotionVoodooArbFFControlThread() {
		if (motionVoodooArbFFControlThread != null) {
			if (!motionVoodooArbFFControlThread.isAlive()) {
				resetMotionVoodooArbFFLambda();
				motionVoodooArbFFControlThread.start();
			}
		} else {
			resetMotionVoodooArbFFLambda();
			motionVoodooArbFFControlThread.start();
		}
	}

	private void resetMotionVoodooArbFFLambda() {
		//Instantiate Motion Magic Thread
		motionVoodooArbFFControlThread = new Thread(() -> {
			ThreadRateControl trc = new ThreadRateControl();
			trc.start();
			while (currentControlMode == MCControlMode.MotionVoodooArbFF) {
				double demandStep = generateMotionVoodooArbFFValue(getPosition(), motionVoodooArbFFDemand, getVelocity(), trc.getDt());
				prevMotionVelocitySetpoint = demandStep;
				demandStep = Math.abs(demandStep) < minSetpointOutput ? 0 : demandStep;
				double arbFF = motionVoodooArbFFLookup.getInterpolated(new InterpolatingDouble(getPosition() - absoluteEncoderOffset)).value;
				mTalonFX.set(ControlMode.Velocity, convertDemandToNativeUnits(MCControlMode.MotionVoodooArbFF, demandStep), DemandType.ArbitraryFeedForward, arbFF);
//				ConsoleReporter.report("ArbFF: " + arbFF + ", OutputDC: " + getMCOutputPercent() + ", Pos: " + getPosition() + ", Spd: " + demandStep);
				trc.doRateControl(10);
			}
		});
	}

	private double generateMotionVoodooArbFFValue(double position, double setpoint, double speed, double dt) {
		double motionVoodooArbFFVelocity = mFeedbackConfig.get(currentSelectedSlot).motionMagicVel;
		double motionVoodooArbFFMaxAccel = mFeedbackConfig.get(currentSelectedSlot).motionMagicAccel;

		double diffErr = setpoint - position;
		double diffSign = Math.copySign(1.0, diffErr);
		diffErr = Math.abs(diffErr);

		if (motionVoodooArbFFVelocity == 0 || motionVoodooArbFFMaxAccel == 0 || diffErr <= allowedClosedLoopError)
			return 0;

		double decelVelocity = Math.sqrt(diffErr * 2.0 / (motionVoodooArbFFMaxAccel * 60.0)) * (motionVoodooArbFFVelocity * 60.0);

		speedNew = prevMotionVelocitySetpoint + (motionVoodooArbFFMaxAccel * dt * diffSign);
		double absoluteSpeed = Math.abs(speedNew);
		speedNew = absoluteSpeed > motionVoodooArbFFVelocity ? Math.copySign(motionVoodooArbFFVelocity, speedNew) : speedNew;

		speedNew = absoluteSpeed <= decelVelocity ? speedNew : Math.copySign(decelVelocity, speedNew);
		speedNew = absoluteSpeed <= minSetpointOutput ? 0 : speedNew;
		return speedNew;
	}

	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {
		runTalonFunctionWithRetry((t) -> mTalonFX.config_kP(currentSelectedSlot, kP, Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.config_kI(currentSelectedSlot, kI, Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.config_kD(currentSelectedSlot, kD, Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.config_kF(currentSelectedSlot, kF, Constants.kCANTimeoutMs));
	}

	@Override
	public void setDFilter(double dFilter) {

	}

	@Override
	public void setIZone(double iZone) {
		runTalonFunctionWithRetry((t) -> mTalonFX.config_IntegralZone(currentSelectedSlot, (int)iZone, Constants.kCANTimeoutMs));
	}

	@Override
	public void setMCIAccum(double iAccum) {
		runTalonFunctionWithRetry((t) -> mTalonFX.setIntegralAccumulator(iAccum, 0, Constants.kCANTimeoutMs));
	}

	@Override
	public void setMaxIAccum(double maxIAccum) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configMaxIntegralAccumulator(currentSelectedSlot, maxIAccum, Constants.kCANTimeoutMs));
	}

	@Override
	public void setMCOpenLoopRampRate(double rampRate) {
		runTalonFunctionWithRetry((t) -> mTalonFX.configOpenloopRamp(rampRate, Constants.kCANTimeoutMs));
	}

	@Override
	public void setMCClosedLoopRampRate(double rampRate) {
		configClosedloopRamp(rampRate);
	}

	@Override
	public void setMotionParameters(double cruiseVel, double cruiseAccel) {
		setMotionParameters(cruiseVel, cruiseAccel, 0);
	}

	public void setMotionParameters(double cruiseVel, double cruiseAccel, int sCurveStrength) {
		mFeedbackConfig.get(currentSelectedSlot).setMotionMagicVel(cruiseVel);
		mFeedbackConfig.get(currentSelectedSlot).setMotionMagicAccel(cruiseAccel);
		mFeedbackConfig.get(currentSelectedSlot).setMotionMagicSCurveStrength(sCurveStrength);
		runTalonFunctionWithRetry((t) -> mTalonFX.configMotionCruiseVelocity(convertRPMToNativeUnits(cruiseVel), Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configMotionAcceleration(convertRPMToNativeUnits(cruiseAccel), Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configMotionSCurveStrength(sCurveStrength, Constants.kTalonRetryCount));
	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);
	}

	@Override
	public void setBrakeCoastMode(MCNeutralMode neutralMode) {
		runTalonFunctionWithRetry((t) -> {
			mTalonFX.setNeutralMode(neutralMode.CTRE());
			return mTalonFX.getLastError();
		});
	}

	@Override
	public void setEncoderPosition(double position) {
		localQuadPosition.setValue(position);
		runTalonFunctionWithRetry((t) -> mTalonFX.setSelectedSensorPosition(convertRotationsToNativeUnits(position), 0, Constants.kCANTimeoutMs));
	}

	@Override
	public void setCurrentLimit(int currentLimit) {
		configCurrentLimit(currentLimit, 0, 0);
	}

	private synchronized void runTalonFunctionWithRetry(Function<Void, ErrorCode> talonCall) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = talonCall.apply(null) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set parameter Talon " + mTalonFX.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void writeToFlash() {

	}

	@Override
	public void disableSoftLimits() {
		runTalonFunctionWithRetry((t) -> mTalonFX.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> mTalonFX.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
	}

	@Override
	public boolean getForwardLimitValue() {
		return forwardLimitCachedValue.getValue();
	}

	@Override
	public boolean getReverseLimitValue() {
		return reverseLimitCachedValue.getValue();
	}

	@Override
	public boolean getForwardLimitRisingEdge() {
		boolean currentInput = getForwardLimitValue();
		boolean retVal = (currentInput != prevForwardLimitVal.get()) && currentInput;
		prevForwardLimitVal.set(currentInput);
		return retVal;
	}

	@Override
	public boolean getReverseLimitRisingEdge() {
		boolean currentInput = getReverseLimitValue();
		boolean retVal = (currentInput != prevReverseLimitVal.get()) && currentInput;
		prevReverseLimitVal.set(currentInput);
		return retVal;
	}

	@Override
	public boolean getForwardLimitFallingEdge() {
		boolean currentInput = getForwardLimitValue();
		boolean retVal = (currentInput != prevForwardLimitVal.get()) && !currentInput;
		prevForwardLimitVal.set(currentInput);
		return retVal;
	}

	@Override
	public boolean getReverseLimitFallingEdge() {
		boolean currentInput = getReverseLimitValue();
		boolean retVal = (currentInput != prevReverseLimitVal.get()) && !currentInput;
		prevReverseLimitVal.set(currentInput);
		return retVal;
	}

	@Override
	public void setControlMode(MCControlMode controlMode) {
		if (getMotionControlMode() != controlMode) {
			switch (controlMode) {
				case PercentOut:
				case Velocity:
				case Current:
					set(controlMode, 0, currentSelectedSlot, 0);
					break;
				case Position:
				case MotionMagic:
				case MotionVoodooArbFF:
					set(controlMode, getPosition(), currentSelectedSlot, 0);
					break;
				case Disabled:
				default:
					mTalonFX.set(ControlMode.Disabled, 0);
					break;
			}
		}
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return currentControlMode;
	}

	@Override
	public void setSetpoint(double setpoint) {
		set(getMotionControlMode(), setpoint, currentSelectedSlot, 0);
	}

	@Override
	public double getActual() {
		switch (currentControlMode) {
			case PercentOut:
//				return getMotorOutputPercent();
			case Position:
			case MotionMagic:
			case MotionVoodooArbFF:
				return getPosition();
			case Velocity:
				return getVelocity();
			case Current:
				return mTalonFX.getSupplyCurrent();
			default:
				return 0;
		}
	}

	@Override
	public double getSetpoint() {
		switch (currentControlMode) {
			case PercentOut:
//				return getMotorOutputPercent();
			case Position:
			case MotionMagic:
				return convertNativeUnitsToRotations(mTalonFX.getClosedLoopTarget());
			case MotionVoodooArbFF:
				return motionVoodooArbFFDemand;
			case Velocity:
				return convertNativeUnitsToRPM((int) mTalonFX.getClosedLoopTarget());
			case Current:
				return mTalonFX.getClosedLoopTarget();
			default:
				return 0;
		}
	}

	@Override
	public double getMCOutputCurrent() {
		return mTalonFX.getSupplyCurrent();
	}

	@Override
	public double getMCOutputPercent() {
		return mTalonFX.getMotorOutputPercent();
	}

	@Override
	public int getMCID() {
		return mTalonFX.getDeviceID();
	}

	@Override
	public boolean isEncoderPresent() {
		return true;
	}

	@Override
	public double getMCInputVoltage() {
		return mTalonFX.getBusVoltage();
	}

	@Override
	public double getMCOutputVoltage() {
		return mTalonFX.getMotorOutputVoltage();
	}

	@Override
	public double getPosition() {
		return convertNativeUnitsToRotations(mTalonFX.getSelectedSensorPosition());
	}

	@Override
	public double getVelocity() {
		return convertNativeUnitsToRPM(mTalonFX.getSelectedSensorVelocity());
	}

	@Override
	public double getSensorUnitsPerRotation() {
		return 2048;
	}

	@Override
	public double getVelocityRPMTimeConversionFactor() {
		return 600;
	}

	@Override
	public double getNativeUnitsOutputRange() {
		return 1023.0;
	}

	@Override
	public double getMCIAccum() {
		return mTalonFX.getIntegralAccumulator();
	}

	@Override
	public DiagnosticMessage hasMotorControllerReset() {
		if (mTalonFX.hasResetOccurred()) {
			String dMsg = "Talon" + mTalonFX.getDeviceID() + "ResetHasOccurred";
			ConsoleReporter.report(dMsg, MessageLevel.DEFCON1);
			runTalonFunctionWithRetry((t) -> mTalonFX.clearStickyFaults(Constants.kLongCANTimeoutMs));
			return new DiagnosticMessage(dMsg);
		}

		return DiagnosticMessage.NO_MSG;
	}

	public class FeedbackConfiguration {
		FeedbackDevice feedbackDevice;
		int remoteDeviceId;
		double closedLoopRampRate;
		double motionMagicVel;
		double motionMagicAccel;
		int motionMagicSCurveStrength;

		FeedbackConfiguration(FeedbackDevice feedbackDevice, int remoteDeviceId, double closedLoopRampRate, double motionMagicVel, double motionMagicAccel, int motionMagicSCurveStrength) {
			this.feedbackDevice = feedbackDevice;
			this.remoteDeviceId = remoteDeviceId;
			this.closedLoopRampRate = closedLoopRampRate;
			this.motionMagicVel = motionMagicVel;
			this.motionMagicAccel = motionMagicAccel;
			this.motionMagicSCurveStrength = motionMagicSCurveStrength;
		}

		FeedbackConfiguration() {
			this(FeedbackDevice.CTRE_MagEncoder_Relative, -1, 0, 0, 0, 0);
		}

		synchronized void setFeedbackDevice(FeedbackDevice feedbackDevice) {
			this.feedbackDevice = feedbackDevice;
		}

		synchronized void setRemoteDeviceId(int remoteDeviceId) {
			this.remoteDeviceId = remoteDeviceId;
		}

		synchronized void setClosedLoopRampRate(double closedLoopRampRate) {
			this.closedLoopRampRate = closedLoopRampRate;
		}

		synchronized void setMotionMagicVel(double motionMagicVel) {
			this.motionMagicVel = motionMagicVel;
		}

		synchronized void setMotionMagicAccel(double motionMagicAccel) {
			this.motionMagicAccel = motionMagicAccel;
		}

		synchronized void setMotionMagicSCurveStrength(int sCurveStrength) {
			this.motionMagicSCurveStrength = sCurveStrength;
		}
	}

	private static class Configuration {
		int CONTROL_FRAME_PERIOD_MS;
		int STATUS_FRAME_GENERAL_1_MS;
		int STATUS_FRAME_FEEDBACK0_2_MS;

		Configuration(int CONTROL_FRAME_PERIOD_MS, int STATUS_FRAME_GENERAL_1_MS, int STATUS_FRAME_FEEDBACK0_2_MS) {
			this.CONTROL_FRAME_PERIOD_MS = CONTROL_FRAME_PERIOD_MS;
			this.STATUS_FRAME_GENERAL_1_MS = STATUS_FRAME_GENERAL_1_MS;
			this.STATUS_FRAME_FEEDBACK0_2_MS = STATUS_FRAME_FEEDBACK0_2_MS;
		}
	}
}

