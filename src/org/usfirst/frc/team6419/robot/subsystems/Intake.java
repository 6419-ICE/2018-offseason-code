package org.usfirst.frc.team6419.robot.subsystems;

import org.usfirst.frc.team6419.robot.commands.HandleIntake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Intake subsystem
 */
public class Intake extends Subsystem {
	
	private DoubleSolenoid intakeSolenoid;
	private VictorSP right0, right1, left0, left1;
	
	/**
	 * Construct a new Intake
	 * @param solenoidPin0
	 * @param solenoidPin1
	 * @param r0Pin
	 * @param r1Pin
	 * @param l0Pin
	 * @param l1Pin
	 */
	public Intake(int solenoidPin0, int solenoidPin1, int r0Pin, int r1Pin, int l0Pin, int l1Pin) {
		// Instantiate and initialize the intake solenoid
		intakeSolenoid = new DoubleSolenoid(solenoidPin0, solenoidPin1);
		intakeSolenoid.set(DoubleSolenoid.Value.kOff);
		
		// Instantiate the intake motor controllers
		right0 = new VictorSP(r0Pin);
		right1 = new VictorSP(r1Pin);
		left0 = new VictorSP(l0Pin);
		left1 = new VictorSP(l1Pin);
		
		// Invert the motors on the right
		left0.setInverted(false);
		left1.setInverted(false);
		right0.setInverted(true);
		right1.setInverted(true);
	}

    public void initDefaultCommand() {
    	setDefaultCommand(new HandleIntake());
    }
    
    /**
     * Set power of the intake motors
     * @param power
     */
    public void setPower(double power) {
    	right0.set(power);
    	right1.set(power);
    	left0.set(power);
    	left1.set(power);
    }
    
    /**
     * Open or close the intake
     * @param closed true -> closed, false -> open
     */
    public void setClosed(boolean closed) {
    	if (closed) {
    		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    	} else {
    		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    	}
    }
    
    /**
     * Toggle between open and closed
     */
    public void toggle() {
    	if (intakeSolenoid.get().equals(DoubleSolenoid.Value.kForward)) {
    		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    	} else {
    		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    	}
    }
    
    /**
     * Depressurize the cylinders
     */
    public void depressurize() {
    	intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }
}

