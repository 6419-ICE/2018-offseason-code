package org.usfirst.frc.team6419.robot;

public class Config {
	public static final double wheelDiameter = 6; // inches
	public static final double driveTalonSpeedThreshold = 0.1;
	public static final int ticksPerRev = 4096;
	public static final double inchesPerTick = wheelDiameter / ticksPerRev;
	public static final double ticksPerInch = ticksPerRev / wheelDiameter;
	public static final double stpMaxSpeed = 0.1 * ticksPerRev * inchesPerTick;
	public static final double stpMaxTickSpeed = 0.1 * ticksPerRev;
	public static final int liftMotorPin = 0;
	public static final int driveTalonEncoderErrorThreshold = 15;
}
