package org.usfirst.frc.team3476.Utility.Control;

import edu.wpi.first.wpilibj.PIDSource;

public class MedianEncoderPair implements PIDSource
{
	private MedianEncoder left, right;
	private boolean useLeft, useRight;
	
	public MedianEncoderPair(MedianEncoder leftin, MedianEncoder rightin)
	{
		left = leftin;
		right = rightin;
		useLeft = true;
		useRight = true;
	}
	
	public MedianEncoderPair(MedianEncoder leftin, MedianEncoder rightin, boolean useLeftin, boolean useRightin)
	{
		left = leftin;
		right = rightin;
		useLeft = useLeftin;
		useRight = useRightin;
	}
	
	public void setUse(boolean useLeftin, boolean useRightin)
	{
		useLeft = useLeftin;
		useRight = useRightin;
	}
	
	@Override
	public double pidGet()
	{
		return getDistance();
	}
	
	public double getDistance()
	{
		if(useLeft && useRight)
		{
			return (left.getDistance() + right.getDistance())/2;
		}
		else if(useLeft)
		{
			return left.getDistance();
		}
		else if(useRight)
		{
			return right.getDistance();
		}
		else
		{
			return 0;
		}
	}
}
