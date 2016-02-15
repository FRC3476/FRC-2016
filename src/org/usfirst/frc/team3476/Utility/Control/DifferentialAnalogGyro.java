package org.usfirst.frc.team3476.Utility.Control;

import java.util.Arrays;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;

public class DifferentialAnalogGyro extends AnalogGyro implements PIDSource
{
	public final int DEFSAMPLES = 5;
	private int samples;
	
	public DifferentialAnalogGyro(int channel)
	{
		super(channel);
		samples = DEFSAMPLES;
	}
	
	public DifferentialAnalogGyro(int channel, int numSamples)
	{
		super(channel);
		samples = numSamples;
	}
	
	public DifferentialAnalogGyro(AnalogInput channel)
	{
		super(channel);
		samples = DEFSAMPLES;
	}
	
	public DifferentialAnalogGyro(AnalogInput channel, int numSamples)
	{
		super(channel);
		samples = numSamples;
	}

	private double lastHeading;
	
	public double calcDiff()
	{
		return get() - lastHeading;
	}
	
	public void resetDiff()
	{
		lastHeading = get();
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
