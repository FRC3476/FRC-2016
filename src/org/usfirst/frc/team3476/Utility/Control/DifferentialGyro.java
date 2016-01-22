package org.usfirst.frc.team3476.Utility.Control;

import java.util.Arrays;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;

public class DifferentialGyro extends AnalogGyro implements PIDSource
{
	public final int DEFSAMPLES = 5;
	private int samples;
	
	public DifferentialGyro(int channel)
	{
		super(channel);
		samples = DEFSAMPLES;
	}
	
	public DifferentialGyro(int channel, int numSamples)
	{
		super(channel);
		samples = numSamples;
	}
	
	public DifferentialGyro(AnalogInput channel)
	{
		super(channel);
		samples = DEFSAMPLES;
	}
	
	public DifferentialGyro(AnalogInput channel, int numSamples)
	{
		super(channel);
		samples = numSamples;
	}

	private double lastHeading;
	
	public double calcDiff()
	{
		return get() - lastHeading;
	}
	
	public void reset()
	{
		lastHeading = get();
	}
	
	public void resetSuper()
	{
		super.reset();
	}
	
	@Override
	public double pidGet()
	{
		return calcDiff();
	}
	
	public double get()
	{
		double[] data = new double[samples];
		
		for(int i = 0; i < data.length; i++)
		{
			data[i] = getAngle();
		}
		
		Arrays.sort(data);
		
		return data[(int)(samples/2)];
	}
}
