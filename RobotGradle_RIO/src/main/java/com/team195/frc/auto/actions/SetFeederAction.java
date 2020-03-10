package com.team195.frc.auto.actions;

import com.team195.frc.subsystems.Intake;

import java.util.function.Supplier;

public class SetFeederAction implements Action {
	private static final Intake mIntake = Intake.getInstance();

	private final Supplier<Boolean> mButtonGetterMethod;
	private final Intake.FeederControlMode mControlMode;

	public SetFeederAction(boolean reverse, Supplier<Boolean> buttonGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mControlMode = reverse ? Intake.FeederControlMode.REVERSE : Intake.FeederControlMode.FORWARD;
	}

	@Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.get());
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {
		mIntake.setFeederControlMode(Intake.FeederControlMode.OFF);
	}

	@Override
	public void start() {
		mIntake.setFeederControlMode(mControlMode);
	}
}
