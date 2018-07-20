package org.usfirst.frc.team6419.robot.core;

import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class PID implements Sendable {
	
	private PIDTunings positive, negative;
	private double setpoint;
	private double dUpdateRate;
	private PriorityQueue<Double> timeList, errorList;
	private double accumulatedError, lastTimeStamp_I, lastError_I;
	private double deadband;
	private boolean atTarget;
	private double pCmd, iCmd, dCmd, fCmd;
	private double total;
	
	private double lastError;
	private double lastCmd;
	
	private String subsystem = "", name = "", msg = "";
	
	public PID(PIDTunings tunings) {
		setTunings(tunings);
		timeList = new PriorityQueue();
		errorList = new PriorityQueue();
	}
	
	public void setTunings(PIDTunings tunings) {
		positive = tunings;
		negative = tunings;
		negative.maxCmd = -positive.maxCmd;
		negative.minCmd = -positive.minCmd;
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
		double error = setpoint - input;
		pCmd = 0; iCmd = 0; dCmd = 0; fCmd = 0;
		double currentTime = Timer.getFPGATimestamp();
		PIDTunings tunings = (error > 0) ? positive : negative;
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
			
			lastError = error;
			
			fCmd = tunings.kF;
			
			total = pCmd + iCmd + dCmd + fCmd;
			if (error > 0) {
				/*if (total > tunings.maxCmd) {
					msg = "Cmd cons+max";
					cmd = tunings.maxCmd;
				} else if (total < tunings.minCmd) {
					msg = "Cmd cons+min";
					cmd = tunings.minCmd;
				} else {
					cmd = total;
				}*/
				//cmd = Math.min(Math.max(pCmd + iCmd + dCmd + fCmd, tunings.minCmd), tunings.maxCmd);
				cmd = Utils.constrain(total, tunings.minCmd, tunings.maxCmd);
			} else {
				/*if (total < tunings.maxCmd) {
					msg = "Cmd cons-max";
					cmd = tunings.maxCmd;
				} else if (total > tunings.minCmd) {
					msg = "Cmd cons-min";
					cmd = tunings.minCmd;
				} else {
					cmd = total;
				}*/
				cmd = Math.max(Math.min(total, tunings.minCmd), tunings.maxCmd);
				//cmd = Utils.constrain(total, tunings.minCmd, tunings.maxCmd);
			}
		} else {
			cmd = 0;
			atTarget = true;
			msg = "AT";
		}
		lastCmd = cmd;
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
	
	@Override
	public String getSubsystem() {
		return subsystem;
	}
	
	@Override
	public String getName() {
		return name;
	}
	
	@Override
	public void setSubsystem(String subsystem) {
		this.subsystem = subsystem;
	}
	
	@Override
	public void setName(String name) {
		this.name = name;
	}
	
	@Override
	public void initSendable(SendableBuilder builder) {
		System.out.println("Initializing PID sendable");
		builder.addDoubleProperty("Setpoint", () -> getSetpoint(), (x) -> setSetpoint(x));
		builder.addDoubleProperty("Error", () -> lastError, null);
		builder.addDoubleProperty("Deadband", () -> getDeadband(), (x) -> setDeadband(x));
		builder.addDoubleProperty("Command", () -> lastCmd, null);
		builder.addBooleanProperty("At Target", () -> atTarget, null);
		builder.addDoubleProperty("P Cmd", () -> pCmd, null);
		builder.addDoubleProperty("I Cmd", () -> iCmd, null);
		builder.addDoubleProperty("D Cmd", () -> dCmd, null);
		builder.addDoubleProperty("D Cmd", () -> dCmd, null);
		builder.addStringProperty("Message", () -> msg, null);
		builder.addDoubleProperty("Total", () -> total, null);
	}
}
