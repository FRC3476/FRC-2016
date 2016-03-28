package org.usfirst.frc.team3476.Utility.Control;

import edu.wpi.first.wpilibj.DigitalInput;

public class OrangeDigitalInput extends DigitalInput implements OrangeDigital//get is supplied in DigitalInput
{
	boolean inverted;
	
	public OrangeDigitalInput(int channel)
	{
		super(channel);
	}
	
	public void setInverted(boolean inverted)
	{
		this.inverted = inverted;
	}
	
	@Override
	public boolean get()
	{
		return inverted ? !super.get() : super.get();
	}
}
