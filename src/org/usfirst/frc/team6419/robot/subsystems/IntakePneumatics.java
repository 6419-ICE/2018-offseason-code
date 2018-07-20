

public class IntakePneumatics extends Subsystem{
	private DoubleSolenoid left, right;
	public IntakePneumatics() {
		left = new DoubleSolenoid(RobotMap.LEFT_SOLENOID_FORWARD_CHANNEL, RobotMap.LEFT_SOLENOID_REVERSE_CHANNEL);
		right = new DoubleSolenoid(RobotMap.RIGHT_SOLENOID_FORWARD_CHANNEL, RobotMap.RIGHT_SOLENOID_REVERSE_CHANNEL);
				
	}
	/**
	 * Switches the position of the intake solenoids
	 * Sets them to out if they are stopped.
	 */
	public void toggle() {
		left.set(left.get() == DoubleSolenoid.kReverse ? DoubleSolenoid.kForward, DoubleSolenoid.kReverse);
		right.set(right.get() == DoubleSolenoid.kReverse ? DoubleSolenoid.kForward, DoubleSolenoid.kReverse);
		
	}
	/**
	 * Stops air pressure to both ends of the solenoid.
	 */
	public void stop() {
		left.set(DoubleSolenoid.kOff);
		right.set(DoubleSolenoid.kOff);
	}
	
	public void setClosed() {
		left.set(DoubleSolenoid.kForward);
		right.set(DoubleSolenoid.kForward);
	}
	public void setOpen() {
		left.set(DoubleSolenoid.kReverse);
		right.set(DoubleSolenoid.kReverse);
	}
	@Override
	public void initDefaultCommand() {
		
	}
	
}
