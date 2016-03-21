package org.usfirst.frc.team3476.Utility.Control;

import edu.wpi.first.wpilibj.AnalogInput;

public class AtoD implements OrangeDigital
{
	private AnalogInput input;
	private double threshold;
	private boolean inverted;
	
	public AtoD(AnalogInput input, double threshold, boolean inverted)
	{
		this.input = input;
		this.threshold = threshold;
		this.inverted = inverted;
	}
	
	public boolean below()
	{
		return input.getAverageVoltage() < threshold;
	}
	
	public boolean above()
	{
		return input.getAverageVoltage() > threshold;
	}
	
	public boolean get()
	{
		if(inverted)
		{
			return below();
		}
		else
		{
			return above();
		}
	}
}
