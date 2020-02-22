package com.team195.lib.drivers.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.team195.frc.constants.Constants;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.DiagnosticMessage;
import com.team195.frc.reporters.MessageLevel;
import com.team195.lib.util.CachedValue;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Function;

public class CKTalonFX extends TalonFX implements TuneableMotorController {
	private int currentSelectedSlot = 0;
	private ArrayList<FeedbackConfiguration> mFeedbackConfig = new ArrayList<>();
	private double prevOutput = Double.MIN_VALUE;
	private final PDPBreaker motorBreaker;
	private AtomicBoolean prevForwardLimitVal = new AtomicBoolean(false);
	private AtomicBoolean prevReverseLimitVal = new AtomicBoolean(false);

	private ArrayList<CKTalonFX> followerTalons = new ArrayList<>();

	private CachedValue<Double> localQuadPosition;

	private CachedValue<Boolean> forwardLimitCachedValue;
	private CachedValue<Boolean> reverseLimitCachedValue;

	private boolean sensorInverted = false;

	private static final Configuration fastMasterConfig = new Configuration(5, 5, 20);
	private static final Configuration normalMasterConfig = new Configuration(10, 10, 20);
	private static final Configuration normalSlaveConfig = new Configuration(10, 100, 100);

	private MCControlMode currentControlMode = MCControlMode.Disabled;  //Force an update

	private double mKa = 0;
	private double mKv = 0;
	private double mVoltageCompSat = 12;

	public double absoluteEncoderOffset = 0;

	private double mGearRatioToOutput = 1;

	private CKTalonFX(int deviceId, PDPBreaker breakerCurrent, Configuration deviceConfig) {
		super(deviceId);
		initializeFeedbackList();
		super.configFactoryDefault();
		motorBreaker = breakerCurrent;
		doDefaultConfig(deviceConfig);
		setBrakeCoastMode(MCNeutralMode.Brake);
		initCachedValues();
	}

	public CKTalonFX(int deviceId, boolean fastMaster, PDPBreaker breakerCurrent) {
		this(deviceId, breakerCurrent, fastMaster ? fastMasterConfig : normalMasterConfig);
		set(MCControlMode.PercentOut, 0, 0, 0);
	}

	public CKTalonFX(int deviceId, CKTalonFX masterTalon, PDPBreaker breakerCurrent) {
		this(deviceId, breakerCurrent, normalSlaveConfig);
		super.follow(masterTalon);
		masterTalon.followerTalons.add(this);
	}

	public boolean configMasterAndSlaves(Function<CKTalonFX, ErrorCode> configMethod) {
		boolean success = runTalonFunctionWithRetry(configMethod, this) == ErrorCode.OK;
		for(CKTalonFX t : followerTalons) {
			success &= runTalonFunctionWithRetry(configMethod, t) == ErrorCode.OK;
		}
		return success;
	}

	private void initCachedValues() {
		localQuadPosition = new CachedValue<>(100, (t) -> convertNativeUnitsToRotations(super.getSensorCollection().getIntegratedSensorPosition() * (sensorInverted ? -1 : 1)));
		forwardLimitCachedValue = new CachedValue<>(150, (t) -> super.getSensorCollection().isFwdLimitSwitchClosed() == 1);
		reverseLimitCachedValue = new CachedValue<>(150, (t) -> super.getSensorCollection().isRevLimitSwitchClosed() == 1);
	}

	public TalonFXSensorCollection getSensorCollection() {
		return super.getSensorCollection();
	}

	public void setLocalQuadPosition(double position) {
		runTalonFunctionWithRetry((t) -> super.getSensorCollection().setIntegratedSensorPosition(convertRotationsToNativeUnits(position), Constants.kCANTimeoutMs));
	}

	@Override
	public synchronized ErrorCode configVoltageCompSaturation(double voltageCompSat) {
		runTalonFunctionWithRetry((t) -> super.configVoltageCompSaturation(voltageCompSat));
		mVoltageCompSat = voltageCompSat;
		return runTalonFunctionWithRetry((t) -> {
			super.enableVoltageCompensation(true);
			return super.getLastError();
		});
	}

	private void doDefaultConfig(Configuration config) {
		runTalonFunctionWithRetry((t) -> super.clearStickyFaults(Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS));
		runTalonFunctionWithRetry((t) -> super.setStatusFramePeriod(StatusFrame.Status_1_General, config.STATUS_FRAME_GENERAL_1_MS, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.STATUS_FRAME_FEEDBACK0_2_MS, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs));
		configCurrentLimit(motorBreaker.value - 10, motorBreaker.value, getMSDurationForBreakerLimit(motorBreaker.value * 2, motorBreaker.value, 8));
		runTalonFunctionWithRetry((t) -> super.configVoltageCompSaturation(mVoltageCompSat));
		runTalonFunctionWithRetry((t) -> {
			super.enableVoltageCompensation(true);
			return super.getLastError();
		});
		runTalonFunctionWithRetry((t) -> super.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs));
	}

	private void initializeFeedbackList() {
		for (int i = 0; i < 4; i++) {
			mFeedbackConfig.add(new FeedbackConfiguration());
		}
	}

	public ErrorCode configCurrentLimit(int continuousCurrentValueA, int thresholdCurrentA, int durationOverThresholdToEnableLimitingMs) {
		return runTalonFunctionWithRetry((t) -> super.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, continuousCurrentValueA, thresholdCurrentA, durationOverThresholdToEnableLimitingMs)));
	}

	public static final SupplyCurrentLimitConfiguration kCurrentLimitOff = new SupplyCurrentLimitConfiguration(false, 0, 0, 0);
	public ErrorCode disableCurrentLimit() {
		return runTalonFunctionWithRetry((t) -> super.configSupplyCurrentLimit(kCurrentLimitOff));
	}

	public void setFeedbackDevice(FeedbackDevice feedbackDevice) {
		mFeedbackConfig.get(currentSelectedSlot).setFeedbackDevice(feedbackDevice);
		mFeedbackConfig.get(currentSelectedSlot).setRemoteDeviceId(-1);
		runTalonFunctionWithRetry((t) -> super.configSelectedFeedbackSensor(feedbackDevice, 0, Constants.kCANTimeoutMs));
	}

	public void setFeedbackDevice(RemoteFeedbackDevice feedbackDevice, int remoteDeviceId) {
		mFeedbackConfig.get(currentSelectedSlot).setFeedbackDevice(feedbackDevice.getFeedbackDevice());
		mFeedbackConfig.get(currentSelectedSlot).setRemoteDeviceId(remoteDeviceId);
		runTalonFunctionWithRetry((t) -> super.configRemoteFeedbackFilter(remoteDeviceId, RemoteSensorSource.TalonSRX_SelectedSensor, feedbackDevice == RemoteFeedbackDevice.RemoteSensor1 ? 1 : 0, Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configSelectedFeedbackSensor(feedbackDevice, 0, Constants.kCANTimeoutMs));
	}

	public void setInverted(InvertType invertType) {
		runTalonFunctionWithRetry((t) -> {
			super.setInverted(invertType);
			return super.getLastError();
		});
	}

	@Override
	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull) {
		mFeedbackConfig.get(currentSelectedSlot).setClosedLoopRampRate(secondsFromNeutralToFull);
		return runTalonFunctionWithRetry((t) -> super.configClosedloopRamp(secondsFromNeutralToFull, Constants.kCANTimeoutMs));
	}

	public ErrorCode configForwardSoftLimitThreshold(double rotations) {
		return runTalonFunctionWithRetry((t) -> super.configForwardSoftLimitThreshold(convertRotationsToNativeUnits(rotations), Constants.kCANTimeoutMs));
	}

	@Override
	public ErrorCode configForwardSoftLimitEnable(boolean enabled) {
		return runTalonFunctionWithRetry((t) -> super.configForwardSoftLimitEnable(enabled, Constants.kCANTimeoutMs));
	}

	public void configReverseSoftLimitThreshold(double rotations) {
		runTalonFunctionWithRetry((t) -> super.configReverseSoftLimitThreshold(convertRotationsToNativeUnits(rotations), Constants.kCANTimeoutMs));
	}

	@Override
	public ErrorCode configReverseSoftLimitEnable(boolean enabled) {
		return runTalonFunctionWithRetry((t) -> super.configReverseSoftLimitEnable(enabled, Constants.kCANTimeoutMs));
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
		super.selectProfileSlot(slotIdx, pidIdx);
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
		}
	}

	private synchronized void setCurrentSlotValue(int slotIdx) {
		currentSelectedSlot = slotIdx;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("General Status Frame 1: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 2: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 3: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 4: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 6: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 7: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 8: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 9: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 10: " + super.getStatusFramePeriod(StatusFrame.Status_10_Targets, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 11: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 12: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 13: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.kLongCANTimeoutMs) + "\r\n");
		sb.append("General Status Frame 14: " + super.getStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, Constants.kLongCANTimeoutMs) + "\r\n");
		return sb.toString();
	}

	// DO NOT OVERRIDE THIS SET
	// THIS WILL BREAK ALL FOLLOWERS, AS IT IS USED INTERNALLY
//	@Deprecated
//	public void set(ControlMode mode, double outputValue) {
//		set(MCControlMode.valueOf(mode), outputValue, currentSelectedSlot, 0);
//	}

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

			} else {
				demand = convertDemandToNativeUnits(controlMode, demand);
				super.set(controlMode.CTRE(), demand, DemandType.ArbitraryFeedForward, arbitraryFeedForward);
				prevOutput = demand + arbitraryFeedForward;
			}
		}
	}

	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {
		runTalonFunctionWithRetry((t) -> super.config_kP(currentSelectedSlot, kP, Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.config_kI(currentSelectedSlot, kI, Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.config_kD(currentSelectedSlot, kD, Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.config_kF(currentSelectedSlot, kF, Constants.kCANTimeoutMs));
	}

	@Override
	public void setDFilter(double dFilter) {

	}

	@Override
	public void setIZone(double iZone) {
		runTalonFunctionWithRetry((t) -> super.config_IntegralZone(currentSelectedSlot, (int)iZone, Constants.kCANTimeoutMs));
	}

	@Override
	public void setMCIAccum(double iAccum) {
		runTalonFunctionWithRetry((t) -> super.setIntegralAccumulator(iAccum, 0, Constants.kCANTimeoutMs));
	}

	@Override
	public void setMaxIAccum(double maxIAccum) {
		runTalonFunctionWithRetry((t) -> super.configMaxIntegralAccumulator(currentSelectedSlot, maxIAccum, Constants.kCANTimeoutMs));
	}

	@Override
	public void setMCOpenLoopRampRate(double rampRate) {
		runTalonFunctionWithRetry((t) -> super.configOpenloopRamp(rampRate, Constants.kCANTimeoutMs));
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
		runTalonFunctionWithRetry((t) -> super.configMotionCruiseVelocity(convertRPMToNativeUnits(cruiseVel), Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configMotionAcceleration(convertRPMToNativeUnits(cruiseAccel), Constants.kCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configMotionSCurveStrength(sCurveStrength, Constants.kTalonRetryCount));
	}

	@Override
	public void setPIDGainSlot(int slotIdx) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);
	}

	@Override
	public void setBrakeCoastMode(MCNeutralMode neutralMode) {
		runTalonFunctionWithRetry((t) -> {
			super.setNeutralMode(neutralMode.CTRE());
			return super.getLastError();
		});
	}

	@Override
	public void setEncoderPosition(double position) {
		localQuadPosition.setValue(position);
		runTalonFunctionWithRetry((t) -> super.setSelectedSensorPosition(convertRotationsToNativeUnits(position), 0, Constants.kCANTimeoutMs));
	}

	@Override
	public void setCurrentLimit(int currentLimit) {
		configCurrentLimit(currentLimit, 0, 0);
	}

	private synchronized ErrorCode runTalonFunctionWithRetry(Function<Void, ErrorCode> talonCall) {
		ErrorCode eCode;
		int retryCounter = 0;

		do {
			eCode = talonCall.apply(null);
		} while(eCode != ErrorCode.OK && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || eCode != ErrorCode.OK)
			ConsoleReporter.report("Failed to set parameter Talon " + super.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);

		return eCode;
	}

	private synchronized ErrorCode runTalonFunctionWithRetry(Function<CKTalonFX, ErrorCode> talonCall, CKTalonFX talon) {
		ErrorCode eCode;
		int retryCounter = 0;

		do {
			eCode = talonCall.apply(talon);
		} while(eCode != ErrorCode.OK && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || eCode != ErrorCode.OK)
			ConsoleReporter.report("Failed to set parameter Talon " + super.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);

		return eCode;
	}

	@Override
	public void writeToFlash() {

	}

	@Override
	public void disableSoftLimits() {
		runTalonFunctionWithRetry((t) -> super.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
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
					super.set(ControlMode.Disabled, 0);
					break;
			}
		}
	}

	public double getArbFFFromVelocity(double requestedVelocity, double currentVelocity, double dt) {
		double requestedAccel = (requestedVelocity - currentVelocity) / dt;
		double arbFFVoltageConversionFactor = (getNativeUnitsOutputRange() / mVoltageCompSat);
		return (requestedVelocity * mKv + requestedAccel * mKa) * arbFFVoltageConversionFactor;
	}

	public synchronized void setMotorConstants(double kV, double kA) {
		mKv = kV;
		mKa = kA;
	}

	public double getKv() {
		return mKv;
	}

	public double getKa() {
		return mKa;
	}

	@Override
	public double getGearRatioToOutputMechanism() {
		return mGearRatioToOutput;
	}

	@Override
	public synchronized void setGearRatioToOutputMechanism(double gearRatioToOutput) {
		mGearRatioToOutput = gearRatioToOutput;
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
			case MotionProfileSW:
			case Velocity:
				return getVelocity();
			case Current:
				return super.getSupplyCurrent();
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
				return convertNativeUnitsToRotations(super.getClosedLoopTarget());
			case MotionVoodooArbFF:
				return 0;
			case MotionProfileSW:
			case Velocity:
				return convertNativeUnitsToRPM((int) super.getClosedLoopTarget());
			case Current:
				return super.getClosedLoopTarget();
			default:
				return 0;
		}
	}

	@Override
	public double getMCOutputCurrent() {
		return super.getSupplyCurrent();
	}

	@Override
	public double getMCOutputPercent() {
		return super.getMotorOutputPercent();
	}

	@Override
	public int getMCID() {
		return super.getDeviceID();
	}

	@Override
	public boolean isEncoderPresent() {
		return true;
	}

	@Override
	public double getMCInputVoltage() {
		return super.getBusVoltage();
	}

	@Override
	public double getMCOutputVoltage() {
		return super.getMotorOutputVoltage();
	}

	@Override
	public double getPosition() {
		return convertNativeUnitsToRotations(super.getSelectedSensorPosition());
	}

	@Override
	public double getVelocity() {
		return convertNativeUnitsToRPM(super.getSelectedSensorVelocity());
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
		return super.getIntegralAccumulator();
	}

	@Override
	public DiagnosticMessage hasMotorControllerReset() {
		if (super.hasResetOccurred()) {
			String dMsg = "Talon" + super.getDeviceID() + "ResetHasOccurred";
			ConsoleReporter.report(dMsg, MessageLevel.DEFCON1);
			runTalonFunctionWithRetry((t) -> super.clearStickyFaults(Constants.kLongCANTimeoutMs));
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