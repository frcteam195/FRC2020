package com.team195.frc.auto.actions;

import com.team195.frc.subsystems.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class SetTurretOpenLoopAction implements Action {
	private static final Turret mTurret = Turret.getInstance();

	BooleanSupplier mButtonGetterMethod;
	DoubleSupplier mAxisGetterMethod;

	public SetTurretOpenLoopAction(BooleanSupplier buttonGetterMethod, DoubleSupplier axisGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mAxisGetterMethod = axisGetterMethod;
	}

	@Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.getAsBoolean());
	}

	@Override
	public void update() {
		mTurret.setTurretPosition(mAxisGetterMethod.getAsDouble() / 2.0);
	}

	@Override
	public void done() {
		mTurret.setTurretControlMode(Turret.TurretControlMode.DISABLED);
	}

	@Override
	public void start() {
		mTurret.setTurretControlMode(Turret.TurretControlMode.OPEN_LOOP);
	}
}
