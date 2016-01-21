package org.usfirst.frc.team3476.Utility.Control;

import org.usfirst.frc.team3476.Communications.Dashcomm;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDDashdataWrapper implements PIDSource
{
	private String key;
	
	public PIDDashdataWrapper(String keyin)
	{
		key = keyin;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource){}

	@Override
	public PIDSourceType getPIDSourceType(){return null;}

	@Override
	public double pidGet()
	{
		double[] max = {0, 0, 0};
		for(double[] target : Dashcomm.getTargetData(key))
		{
			if(target[2] > max[2])
			{
				max = target;
			}
		}
		return max[0];
	}

}
