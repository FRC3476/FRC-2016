package org.usfirst.frc.team3476.Utility.Control;

import edu.wpi.first.wpilibj.AnalogInput;

public class AtoD
{
	private AnalogInput input;
	private double threshold;
	
	public AtoD(AnalogInput input, double threshold)
	{
		this.input = input;
		this.threshold = threshold;
	}
	
	public boolean below()
	{
		return input.getAverageVoltage() < threshold;
	}
	
	public boolean above()
	{
		return input.getAverageVoltage() > threshold;
	}
}
