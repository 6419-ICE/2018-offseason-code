package org.usfirst.frc.team6419.robot;

public class Config {
	public static final double wheelDiameter = 6; // inches
	public static final double wheelCircumference = Math.PI * wheelDiameter;
	public static final double driveTalonSpeedThreshold = 0.1;
	public static final int ticksPerRev = 4096;
	public static final double inchesPerTick = wheelCircumference / ticksPerRev;
	public static final double ticksPerInch = ticksPerRev / wheelCircumference;
	public static final double stpMaxSpeed = 0.1 * ticksPerRev * inchesPerTick;
	public static final double stpMaxTickSpeed = 0.1 * ticksPerRev;
	public static final int liftMotorPin = 0;
	public static final int driveTalonEncoderErrorThreshold = 1500;
	public static final int intakeSolenoidPin0 = 0, intakeSolenoidPin1 = 3;
	public static final int intakeR0 = 1, intakeR1 = 2, intakeL0 = 3, intakeL1 = 4;
}
