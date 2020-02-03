package com.team195.frc.ckcoprocessor;

public final class CoProcessorMain {
	private CoProcessorMain() {
	}

	public static void main(String... args) {
		CKCoprocessorTimedRobot.startRobot(CKCoprocessorRobot::new);
	}
}
