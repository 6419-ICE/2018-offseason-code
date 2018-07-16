package org.usfirst.frc.team6419.robot.core;

import java.util.PriorityQueue;
import edu.wpi.first.wpilibj.Timer;

public class PID {
	
	private PIDTunings positive, negative;
	private double setpoint;
	private double dUpdateRate;
	private PriorityQueue<Double> timeList, errorList;
	private double accumulatedError, lastTimeStamp_I, lastError_I;
	private double deadband;
	private boolean atTarget;
	
	public PID(PIDTunings tunings) {
		setTunings(tunings);
		timeList = new PriorityQueue();
		errorList = new PriorityQueue();
	}
	
	public void setTunings(PIDTunings tunings) {
		positive = tunings;
		negative = tunings;
		negative.maxCmd = -negative.maxCmd;
		negative.minCmd = -negative.minCmd;
	}
	
	public void setPositiveTunings(PIDTunings tunings) {
		positive = tunings;
	}
	
	public void setNegativeTunings(PIDTunings tunings) {
		negative = tunings;
	}
	
	/**
	 * Calculates the PID output using the input.
	 * @param input the sensor value to use.
	 * @return the output from the PID.
	 */
	public double update(double input) {
		double cmd = 0;
		double error = input - setpoint;
		double pCmd = 0, iCmd = 0, dCmd = 0, fCmd = 0;
		double currentTime = Timer.getFPGATimestamp();
		PIDTunings tunings = (error < 0) ? negative : positive;
		if (Math.abs(error) > deadband) {
			atTarget = false;
			pCmd = tunings.kP * error;
			
			if (lastTimeStamp_I > 0) {
				accumulatedError += (lastError_I + error) * (currentTime - lastTimeStamp_I) / 2.0;
				iCmd = tunings.kI * accumulatedError;
			}
			lastTimeStamp_I = currentTime;
			lastError_I = error;
			
			if (timeList.size() > 0) {
				double oldestTime = timeList.peek();
				double oldestError = errorList.peek();
				double deltaT = currentTime - oldestTime;
				
				double rateChange = (error - oldestError) / deltaT;
				
				if (deltaT > dUpdateRate) {
					timeList.remove();
					errorList.remove();
				}
				
				dCmd = tunings.kD * rateChange;
			}
			timeList.add(currentTime);
			errorList.add(error);
			
			fCmd = tunings.kF;
			
			if (error > 0) {
				cmd = Math.min(Math.max(pCmd + iCmd + dCmd + fCmd, tunings.minCmd), tunings.maxCmd);
			} else {
				cmd = Math.max(Math.min(pCmd + iCmd + dCmd + fCmd, tunings.minCmd), tunings.maxCmd);
			}
		} else {
			cmd = 0;
			atTarget = true;
		}
		return cmd;
	}

	public double getSetpoint() {
		return setpoint;
	}

	public double getdUpdateRate() {
		return dUpdateRate;
	}

	public double getDeadband() {
		return deadband;
	}

	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}

	public void setdUpdateRate(double dUpdateRate) {
		this.dUpdateRate = dUpdateRate;
	}

	public void setDeadband(double deadband) {
		this.deadband = deadband;
	}
	/**
	 * Clears the times and errors from the queue.
	 */
	public void reset() {
		timeList.clear();
		errorList.clear();
		dumpIntegrator();
	}
	
	public void dumpIntegrator() {
		lastTimeStamp_I = -1;
		lastError_I = 0;
		accumulatedError = 0;
	}
}
