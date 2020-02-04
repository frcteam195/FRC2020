package frc.robot.motorcontrol;

import com.ctre.phoenix.motorcontrol.ControlMode;

import java.util.HashMap;

public enum MCControlMode {
	PercentOut(0) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.PercentOutput;
		}
	},
	Position(1) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.Position;
		}
	},
	Velocity(2) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.Velocity;
		}
	},
	Current(3) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.Current;
		}
	},
	Voltage(4) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.Disabled;
		}
	},
	MotionMagic(5) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.MotionMagic;
		}
	},
	SmartVelocity(6) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.Velocity;
		}
	},
	MotionVoodooArbFF(7) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.Velocity;
		}
	},
	MotionProfile(8) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.MotionProfile;
		}
	},
	MotionProfileSW(8) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.Velocity;
		}
	},
	Disabled(15) {
		@Override
		public ControlMode CTRE() {
			return ControlMode.Disabled;
		}
	};

	// Keep static maps to do fast lookup on control types via int and other motor library types
	private static HashMap<Integer, MCControlMode> intLookupMap = new HashMap<>();
	private static HashMap<ControlMode, MCControlMode> ctreLookupMap = new HashMap<>();

	static {
		for (MCControlMode type : MCControlMode.values()) {
			intLookupMap.put(type.value, type);
		}

		ctreLookupMap.put(ControlMode.PercentOutput, PercentOut);
		ctreLookupMap.put(ControlMode.Position, Position);
		ctreLookupMap.put(ControlMode.Velocity, Velocity);
		ctreLookupMap.put(ControlMode.Current, Current);
		ctreLookupMap.put(ControlMode.MotionMagic, MotionMagic);
		ctreLookupMap.put(ControlMode.MotionProfile, MotionProfile);
	}

	public final int value;

	MCControlMode(int initValue) {
		this.value = initValue;
	}

	public static MCControlMode valueOf(Object value) {
		MCControlMode retval = null;

		if (value instanceof ControlMode) {
			retval = ctreLookupMap.get(value);
		} else if (value instanceof Integer) {
			retval = intLookupMap.get(value);
		} else if (value instanceof Double) {
			retval = intLookupMap.get((int) ((double) value));
		}

		if (retval != null)
			return retval;
		return Disabled;
	}

	public abstract ControlMode CTRE();

	@Override
	public String toString() {
		return valueOf(value).name();
	}
}