package org.usfirst.frc.team3476.Utility.Control;

import org.usfirst.frc.team3476.Communications.Dashcomm;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDDashdataWrapper implements PIDSource
{
	public enum Data {VISIONX};
	private Data type;
	
	public PIDDashdataWrapper(Data typein)
	{
		type = typein;
	}
	
	@Override
	public double pidGet()
	{
		switch(type)
		{
			//Returns only the x coordinate
			case VISIONX:
				double[][] data = Dashcomm.getTargetData();//[target][Dist,X,Y]
				return data[0][1];
				
			default:
				return 0;
		}
	}
	
	public boolean checkFrame()
	{
		boolean newframe = Dashcomm.getBoolean("data/newframe", false);
		if(newframe)
		{
			Dashcomm.putBoolean("data/newframe", false);
			return true;
		}
		return false;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource){}

	@Override
	public PIDSourceType getPIDSourceType(){return PIDSourceType.kDisplacement;}

}
