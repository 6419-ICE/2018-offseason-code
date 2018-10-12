package org.usfirst.frc.team6419.robot.subsystems;

import com.analog.adis16448.ADIS16448_IMU;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 *
 */
public class IMU extends Subsystem implements Sendable {
	
	private ADIS16448_IMU imu;
	private double lastTimestamp;
	private double resetOffset;
	
	public IMU() {
		imu = new ADIS16448_IMU();
		resetOffset = 0;
		reset();
		calibrate();
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    private void calculate() {
    	double dT = Timer.getFPGATimestamp() - lastTimestamp;
    	
    }
    
    public void reset() {
    	imu.reset();
    }
    
    public void calibrate() {
    	imu.calibrate();
    }
    
    public double getHeading() {
    	return Math.toRadians(imu.getAngleZ());
    }
    
    public void initSendable(SendableBuilder builder) {
    	builder.addDoubleProperty("Yaw", () -> getHeading(), null);
    	super.initSendable(builder);
    }
}

