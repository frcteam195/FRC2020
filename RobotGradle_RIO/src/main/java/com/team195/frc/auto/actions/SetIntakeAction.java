package com.team195.frc.auto.actions;

import com.team195.frc.subsystems.Intake;

import java.util.function.Supplier;

public class SetIntakeAction implements Action {
	private static final Intake mIntake = Intake.getInstance();

	private final Supplier<Boolean> mButtonGetterMethod;
	private final Intake.IntakeControlMode mControlMode;

	public SetIntakeAction(boolean reverse, Supplier<Boolean> buttonGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mControlMode = reverse ? Intake.IntakeControlMode.REVERSE : Intake.IntakeControlMode.FORWARD;
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
		mIntake.setIntakeControlMode(Intake.IntakeControlMode.OFF);
	}

	@Override
	public void start() {
		mIntake.setIntakeControlMode(mControlMode);
	}
}
