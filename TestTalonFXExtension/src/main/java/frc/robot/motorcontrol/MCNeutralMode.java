package frc.robot.motorcontrol;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import java.util.HashMap;

public enum MCNeutralMode {
	Coast(0) {
		@Override
		public NeutralMode CTRE() {
			return NeutralMode.Coast;
		}
	},
	Brake(1) {
		@Override
		public NeutralMode CTRE() {
			return NeutralMode.Brake;
		}
	};

	public final int value;
	MCNeutralMode(int initValue)
	{
		this.value = initValue;
	}

	private static HashMap<Integer, MCNeutralMode> intLookupMap = new HashMap<>();
	private static HashMap<NeutralMode, MCNeutralMode> ctreLookupMap = new HashMap<>();
	static {
		for (MCNeutralMode type : MCNeutralMode.values()) {
			intLookupMap.put(type.value, type);
		}

		ctreLookupMap.put(NeutralMode.Coast, Coast);
		ctreLookupMap.put(NeutralMode.Brake, Brake);
	}

	public static MCNeutralMode valueOf(Object value) {
		MCNeutralMode retval = null;

		if (value instanceof NeutralMode) {
			retval = ctreLookupMap.get(value);
		} else if (value instanceof Integer) {
			retval = intLookupMap.get(value);
		} else if (value instanceof Double) {
			retval = intLookupMap.get((int) ((double) value));
		}

		if (retval != null)
			return retval;
		return Coast;
	}


	public abstract NeutralMode CTRE();
}
