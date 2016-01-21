package org.usfirst.frc.team3476.Utility;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.PIDSource;

public class RunningAverage implements PIDSource
{
	ArrayList<Double> values;
	int numValues;
	double runningAverage;
	
	public RunningAverage(int numValuesIn)
	{
		values = new ArrayList<Double>();
		numValues = numValuesIn;
		runningAverage = 0;
	}
	
	public double getAverage()
	{
		return runningAverage;
	}
	
	public void reset()
	{
		values = new ArrayList<Double>();
		runningAverage = 0;
	}
	
	public void addValue(double value)
	{
		values.add(value);
		if(values.size() > numValues)
		{
			values.remove(0);
		}
		calcAverage();
	}
	
	private void calcAverage()
	{
		double sum = 0;
		for(double val : values)
		{
			sum += val;
		}
		runningAverage = sum/values.size();
	}

	@Override
	public double pidGet()
	{
		return getAverage();
	}
}
