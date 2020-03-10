package com.team195.frc.auto.actions;

import com.team195.frc.subsystems.Drive;
import com.team254.lib.util.DriveSignal;

import java.util.function.Function;
import java.util.function.Supplier;

public class SetOpenLoopDriveAction implements Action {
	private static final Drive mDrive = Drive.getInstance();

	Supplier<Boolean> mButtonGetterMethod;
	Supplier<Double> mAxisLeftGetterMethod;
	Supplier<Double> mAxisRightGetterMethod;

	public SetOpenLoopDriveAction(Supplier<Boolean> buttonGetterMethod, Supplier<Double> axisLeftGetterMethod, Supplier<Double> axisRightGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mAxisLeftGetterMethod = axisLeftGetterMethod;
		mAxisRightGetterMethod = axisRightGetterMethod;
	}

	@Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.get());
	}

	@Override
	public void update() {
		mDrive.setClimbLeft(mAxisLeftGetterMethod.get());
		mDrive.setClimbRight(mAxisRightGetterMethod.get());
	}

	@Override
	public void done() {
		mDrive.setOpenLoop(new DriveSignal(0, 0));
	}

	@Override
	public void start() {
		mDrive.setClimbLeft(0);
		mDrive.setClimbRight(0);
	}
}
