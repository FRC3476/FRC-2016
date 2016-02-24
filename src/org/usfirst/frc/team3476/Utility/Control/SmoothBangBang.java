package org.usfirst.frc.team3476.Utility.Control;

import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.RunningAverage;

public class SmoothBangBang extends BangBang
{
	private RunningAverage average;
	private int samples;
	public SmoothBangBang(double[] outputrangein, int samplesin)
	{
		super(outputrangein);
		samples = samplesin;
		average = new RunningAverage(samples);
		
		double defval = 0;
		
		if(!OrangeUtility.inRange(defval, outputrangein))
		{
			defval = (outputrangein[0] + outputrangein[1])/2;
		}
		
		for(int i = 0; i < samples; i++)
		{
			average.addValue(defval);
		}
	}
	
	@Override
	protected double run(double process)
	{
		average.addValue(super.run(process));
		return average.getAverage();
	}
	
	@Override
	public boolean equals(Object other)
	{
		return this.samples == ((SmoothBangBang)other).samples && super.equals(other);
	}
}
