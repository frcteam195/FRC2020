package com.team195.frc.auto.actions;

import com.team195.frc.constants.CalConstants;
import com.team195.frc.subsystems.Turret;
import com.team195.lib.util.TurretHelper;

import java.util.function.Function;

public class SetTurretPositionJoystickAction implements Action {
	private static final Turret mTurret = Turret.getInstance();

	private Function<Void, Boolean> mButtonGetterMethod;
	private Function<Void, Double> mAxisGetterMethod;

	private double mRotationAcc = 0;

	private static final double kJoystickStep = 1.5;
	private static final double kMaxForwardDeg = TurretHelper.convertRotationsToTurretDegrees(CalConstants.kTurretForwardSoftLimit);
	private static final double kMaxReverseDeg = TurretHelper.convertRotationsToTurretDegrees(CalConstants.kTurretReverseSoftLimit);

	public SetTurretPositionJoystickAction(Function<Void, Boolean> buttonGetterMethod, Function<Void, Double> axisGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mAxisGetterMethod = axisGetterMethod;
	}

	@Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.apply(null));
	}

	@Override
	public void update() {
		mRotationAcc += -mAxisGetterMethod.apply(null) * kJoystickStep;
		mRotationAcc = Math.max(Math.min(mRotationAcc, kMaxForwardDeg), kMaxReverseDeg);
		mTurret.setTurretPosition(TurretHelper.convertTurretDegreesToRotations(mRotationAcc));
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mRotationAcc = TurretHelper.convertRotationsToTurretDegrees(mTurret.getSetpoint());
	}
}
