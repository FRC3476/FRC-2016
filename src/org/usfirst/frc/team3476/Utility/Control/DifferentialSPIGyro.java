package org.usfirst.frc.team3476.Utility.Control;

import java.util.Arrays;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;

public class DifferentialSPIGyro extends ADXRS450_Gyro implements PIDSource
{
	public final int DEFSAMPLES = 5;
	private int samples;
	
	public DifferentialSPIGyro(SPI.Port port)
	{
		super(port);
		samples = DEFSAMPLES;
	}
	
	public DifferentialSPIGyro(SPI.Port port, int numSamples)
	{
		super(port);
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
