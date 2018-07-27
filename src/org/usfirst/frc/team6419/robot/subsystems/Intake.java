package org.usfirst.frc.team6419.robot.subsystems;

import org.usfirst.frc.team6419.robot.commands.HandleIntake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {
	
	private DoubleSolenoid intakeSolenoid;
	private VictorSP right0, right1, left0, left1;
	
	public Intake(int solenoidPin0, int solenoidPin1, int r0Pin, int r1Pin, int l0Pin, int l1Pin) {
		intakeSolenoid = new DoubleSolenoid(solenoidPin0, solenoidPin1);
		
		intakeSolenoid.set(DoubleSolenoid.Value.kOff);
		
		// One side's probably going to need inversion
		right0 = new VictorSP(r0Pin);
		right1 = new VictorSP(r1Pin);
		left0 = new VictorSP(l0Pin);
		left1 = new VictorSP(l1Pin);
		left0.setInverted(false);
		left1.setInverted(false);
		right0.setInverted(true);
		right1.setInverted(true);
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new HandleIntake());
    }
    
    public void setPower(double power) {
    	right0.set(power);
    	right1.set(power);
    	left0.set(power);
    	left1.set(power);
    }
    
    public void setClosed(boolean closed) {
    	if (closed) {
    		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    	} else {
    		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    	}
    }
    
    public void toggle() {
    	if (intakeSolenoid.get().equals(DoubleSolenoid.Value.kForward)) {
    		intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    	} else {
    		intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    	}
    }
    
    /**
     * Depressurize the cylinders
     * In other words, the intake can flop around freely
     */
    public void depressurize() {
    	intakeSolenoid.set(DoubleSolenoid.Value.kOff);
    }
}

