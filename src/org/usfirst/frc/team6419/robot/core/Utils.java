package org.usfirst.frc.team6419.robot.core;

public class Utils {

	public static boolean withinRange(double x, double min, double max) {
		return x >= min && x <= max;
	}
	/**
	 * Returns 0 if the input is too minimal of magnitude to affect the system.
	 * @param x
	 * @param range
	 * @return
	 */
	public static double applyDeadband(double x, double range) {
		if (withinRange(x, -0.5 * range, 0.5 * range)) {
			return 0;
		}
		return x;
	}
	
	public static double constrain(double x, double min, double max) {
		return Math.min(Math.max(x, min), max);
	}
	
	public static double ticksToInches(int ticks, int ticksPerRev, double wheelDiameter) {
		return (wheelDiameter / (double) ticksPerRev) * (double) ticks;
	}
	
	public static int inchesToTicks(double inches, int ticksPerRev, double wheelDiameter) {
		return (int) ((inches / wheelDiameter) * ticksPerRev);
	}
}
