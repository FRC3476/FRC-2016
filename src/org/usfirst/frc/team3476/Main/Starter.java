package org.usfirst.frc.team3476.Main;

import org.usfirst.frc.team3476.Communications.OrangeCamera;
import org.usfirst.frc.team3476.Communications.OrangeCamera.ExposureControl;
import org.usfirst.frc.team3476.ScriptableAuto.Main;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Starter extends SafeTask
{
	private boolean flushed = false, NTready = false, mainInitd = false, cameradone = false;
    private Main main;
    private String year;
	private Subsystem[] systems;
	
    public Starter(Main main_in, String yearin, Subsystem[] systemsin)
    {
    	main = main_in;
    }
    
	@Override
	protected void action() 
	{
		if(!flushed)
		{
			NetworkTable.flush();
			flushed = true;
		}
		else if(!NTready)
		{
			NTInit();//Sets NTready
		}
		else if(!mainInitd)
		{
			main = new Main(year, systems);
		}
		else if(!cameradone)
		{
			cameraInit();
			cameradone = true;
		}
		else
		{
			super.terminate();
		}
	}
	
	public boolean importantDone()
	{
		return flushed && NTready && mainInitd;
	}
	
	private void NTInit()
    {
    	if(NetworkTable.connections().length > 0)
    	{
    		NTready = true;
    	}
    }
	
	private void cameraInit()
    {
    	OrangeCamera cam = new OrangeCamera("axis-camera.local");
    	cam.writeExposureControl(ExposureControl.kHold);
    	cam.writeExposure(0);
    }
}
