package com.team195.frc.auto.autonomy;

import com.team195.frc.auto.actions.*;
import com.team195.frc.subsystems.*;

import java.util.ArrayList;

public class AutomatedActions {

	public static AutomatedAction fullyAutomatedTest() {
		ArrayList<Action> actionArrayList = new ArrayList<>();

		return AutomatedAction.fromAction(new SeriesAction(actionArrayList), 100, Turret.getInstance(), Drive.getInstance());
	}

}