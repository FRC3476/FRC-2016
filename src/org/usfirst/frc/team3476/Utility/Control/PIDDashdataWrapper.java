package org.usfirst.frc.team3476.Utility.Control;

import java.util.Arrays;

import org.usfirst.frc.team3476.Communications.Dashcomm;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDDashdataWrapper implements PIDSource
{
	public enum Data {VISIONX};
	private Data type;
	private boolean stale;
	private long lastDataTime;
	
	public PIDDashdataWrapper(Data typein)
	{
		type = typein;
		stale = true;
		lastDataTime = System.nanoTime();
	}
	
	@Override
	public double pidGet()
	{
		switch(type)
		{
			//Returns only the x coordinate
			case VISIONX:
				return getClosestX();
				
			default:
				return 0;
		}
	}
	
	public double getClosestX()
	{
		return Dashcomm.getTargetData()[0][1];//[target][Dist,X,Y]
	}
	
	public double getClosestDist()
	{
		return Dashcomm.getTargetData()[0][0];//[target][Dist,X,Y]
	}
	
	public boolean checkFrameDouble()
	{
		boolean newframe = Dashcomm.getBoolean("data/newframe", false);
		if(newframe)
		{
			Dashcomm.putBoolean("data/newframe", false);
			return true;
		}
		return false;
	}
	
	public boolean checkFrame()
	{
		boolean newframe = Dashcomm.getBoolean("data/newframe", false);
		if(newframe)
		{
			return true;
		}
		return false;
	}
	
	public boolean targetAvailable()
	{
		double[][] data = Dashcomm.getTargetData();
		boolean datagood = !Arrays.deepEquals(data, new double[][]{{Double.NaN, Double.NaN, Double.NaN}});
		stale = System.nanoTime() - lastDataTime > 200E6;//200 ms in ns
		lastDataTime = System.nanoTime();
		if(stale)
		{
			if(datagood)
			{
				stale = false;
			}
		}
		return datagood;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource){}

	@Override
	public PIDSourceType getPIDSourceType(){return PIDSourceType.kDisplacement;}

}
