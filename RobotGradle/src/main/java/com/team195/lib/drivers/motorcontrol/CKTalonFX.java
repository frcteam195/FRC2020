package com.team195.lib.drivers.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team195.frc.constants.Constants;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.DiagnosticMessage;
import com.team195.frc.reporters.MessageLevel;

import java.util.function.Function;

public class CKTalonFX extends TalonFX implements TuneableMotorController {

	private static final Configuration fastMasterConfig = new Configuration(5, 5, 20);
	private static final Configuration normalMasterConfig = new Configuration(10, 10, 20);
	private static final Configuration normalSlaveConfig = new Configuration(10, 100, 100);
	private final PDPBreaker motorBreaker;

	private CKTalonFX(int deviceId, PDPBreaker breakerCurrent, Configuration deviceConfig) {
		super(deviceId);
		super.configFactoryDefault();
		motorBreaker = breakerCurrent;
		doDefaultConfig(deviceConfig);
		setBrakeCoastMode(MCNeutralMode.Brake);
	}

	public CKTalonFX(int deviceId, boolean fastMaster, PDPBreaker breakerCurrent) {
		this(deviceId, breakerCurrent, fastMaster ? fastMasterConfig : normalMasterConfig);
		set(MCControlMode.PercentOut, 0, 0, 0);
	}

	public CKTalonFX(int deviceId, CKTalonFX masterTalon, PDPBreaker breakerCurrent, boolean inverted) {
		this(deviceId, breakerCurrent, normalSlaveConfig);
		super.follow(masterTalon);
		super.setInverted(inverted);
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
		runTalonFunctionWithRetry((t) -> super.configVoltageCompSaturation(12));
		runTalonFunctionWithRetry((t) -> {
			super.enableVoltageCompensation(true);
			return super.getLastError();
		});
		runTalonFunctionWithRetry((t) -> super.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs));
		runTalonFunctionWithRetry((t) -> super.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs));
	}

	//TODO: Implement new current limiting API
	public void configCurrentLimit(int continuousCurrentValueA, int thresholdCurrentA, int durationOverThresholdToEnableLimitingMs) {
//		runTalonFunctionWithRetry((t) -> super.configContinuousCurrentLimit(continuousCurrentValueA, Constants.kLongCANTimeoutMs));
//		runTalonFunctionWithRetry((t) -> super.configPeakCurrentLimit(thresholdCurrentA, Constants.kLongCANTimeoutMs));
//		runTalonFunctionWithRetry((t) -> super.configPeakCurrentDuration(durationOverThresholdToEnableLimitingMs, Constants.kLongCANTimeoutMs));
//		runTalonFunctionWithRetry((t) -> {
//			super.enableCurrentLimit(true);
//			return super.getLastError();
//		});
	}

	private synchronized void runTalonFunctionWithRetry(Function<Void, ErrorCode> talonCall) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = talonCall.apply(null) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to set parameter Talon " + super.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);
	}

	@Override
	public void set(MCControlMode controlMode, double demand, int slotIdx, double arbitraryFeedForward) {

	}

	@Override
	public void setPIDF(double kP, double kI, double kD, double kF) {

	}

	@Override
	public void setDFilter(double dFilter) {

	}

	@Override
	public void setIZone(double iZone) {

	}

	@Override
	public void setMCIAccum(double iAccum) {

	}

	@Override
	public void setMaxIAccum(double maxIAccum) {

	}

	@Override
	public void setMCOpenLoopRampRate(double rampRate) {

	}

	@Override
	public void setMCClosedLoopRampRate(double rampRate) {

	}

	@Override
	public void setMotionParameters(double cruiseVel, double cruiseAccel) {

	}

	@Override
	public void setPIDGainSlot(int slotIdx) {

	}

	@Override
	public void setBrakeCoastMode(MCNeutralMode neutralMode) {

	}

	@Override
	public void setEncoderPosition(double position) {

	}

	@Override
	public void setCurrentLimit(int currentLimit) {

	}

	@Override
	public void writeToFlash() {

	}

	@Override
	public void disableSoftLimits() {

	}

	@Override
	public boolean getForwardLimitValue() {
		return false;
	}

	@Override
	public boolean getReverseLimitValue() {
		return false;
	}

	@Override
	public boolean getForwardLimitRisingEdge() {
		return false;
	}

	@Override
	public boolean getReverseLimitRisingEdge() {
		return false;
	}

	@Override
	public boolean getForwardLimitFallingEdge() {
		return false;
	}

	@Override
	public boolean getReverseLimitFallingEdge() {
		return false;
	}

	@Override
	public void setControlMode(MCControlMode controlMode) {

	}

	@Override
	public void setSetpoint(double setpoint) {

	}

	@Override
	public double getActual() {
//		switch (currentControlMode) {
//			case PercentOut:
////				return getMotorOutputPercent();
//			case Position:
//			case MotionMagic:
//			case MotionVoodooArbFF:
//				return getPosition();
//			case Velocity:
//				return getVelocity();
//			case Current:
//				return mTalonSRX.getOutputCurrent();
//			default:
//				return 0;
//		}
		return 0;
	}

	@Override
	public double getSetpoint() {
//		switch (currentControlMode) {
//			case PercentOut:
////				return getMotorOutputPercent();
//			case Position:
//			case MotionMagic:
//				return convertNativeUnitsToRotations(mTalonSRX.getClosedLoopTarget());
//			case MotionVoodooArbFF:
//				return motionVoodooArbFFDemand;
//			case Velocity:
//				return convertNativeUnitsToRPM((int)mTalonSRX.getClosedLoopTarget());
//			case Current:
//				return mTalonSRX.getClosedLoopTarget();
//			default:
//				return 0;
//		}
		return 0;
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
	public double getMCOutputCurrent() {
		return 0;
	}

	@Override
	public double getMCInputVoltage() {
		return 0;
	}

	@Override
	public double getMCOutputVoltage() {
		return 0;
	}

	@Override
	public double getMCOutputPercent() {
		return 0;
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
	public DiagnosticMessage hasMotorControllerReset() {
		if (hasResetOccurred()) {
			String dMsg = "Talon" + getDeviceID() + "ResetHasOccurred";
			ConsoleReporter.report(dMsg, MessageLevel.DEFCON1);
			runTalonFunctionWithRetry((t) -> super.clearStickyFaults(Constants.kLongCANTimeoutMs));
			return new DiagnosticMessage(dMsg);
		}

		return DiagnosticMessage.NO_MSG;
	}

	@Override
	public MCControlMode getMotionControlMode() {
		return null;
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
