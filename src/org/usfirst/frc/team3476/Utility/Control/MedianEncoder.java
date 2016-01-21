package org.usfirst.frc.team3476.Utility.Control;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;

public class MedianEncoder extends Encoder implements PIDSource
{
	public final int DEFSAMPLES = 5;
	private int samples;
	
	public MedianEncoder(int arg0, int arg1, boolean arg2, EncodingType arg3)
	{
		super(arg0, arg1, arg2, arg3);
	}
	
	public MedianEncoder(int arg0, int arg1, boolean arg2, EncodingType arg3, int numSamples)
	{
		super(arg0, arg1, arg2, arg3);
		samples = numSamples;
	}
	
	@Override
	public double getDistance()
	{
		double[] data = new double[samples];
		
		for(int i = 0; i < data.length; i++)
		{
			data[i] = super.getDistance();
		}
		
		Arrays.sort(data);
		
		return data[(int)(samples/2)];
	}
	
	@Override
	public double pidGet()
	{
		return getDistance();
	}
}
