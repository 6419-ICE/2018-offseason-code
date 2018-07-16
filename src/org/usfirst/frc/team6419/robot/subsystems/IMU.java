package org.usfirst.frc.team6419.robot.subsystems;

import com.analog.adis16448.ADIS16448_IMU;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IMU extends Subsystem {
	
	private ADIS16448_IMU imu;
	
	public IMU() {
		imu = new ADIS16448_IMU();
		reset();
		calibrate();
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void reset() {
    	imu.reset();
    }
    
    public void calibrate() {
    	imu.calibrate();
    }
    
    public double getHeading() {
    	return Math.toRadians(imu.getAngle());
    }
}

