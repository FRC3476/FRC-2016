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
				boolean found = false;
				double[] max = {0, 0, 0};
				for(double[] target : Dashcomm.getTargetData())
				{
					if(target[2] > max[2])
					{
						found = true;
						max = target;
					}
				}
				if(!found)
				{
					return Double.NaN;
				}
				else
				{
					return max[0];
				}
				
			default:
				return 0;
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource){}

	@Override
	public PIDSourceType getPIDSourceType(){return PIDSourceType.kDisplacement;}

}
