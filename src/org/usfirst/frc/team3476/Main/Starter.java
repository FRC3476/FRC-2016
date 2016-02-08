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
	
    public Starter(Main main_in, String yearin, Subsystem[] systemsin, int minTime)
    {
    	super(minTime);
    	main = main_in;
    	year = yearin;
    	systems = systemsin;
    }
    
	@Override
	protected synchronized void action()
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
			main.initialize(year, systems);
			mainInitd = true;
			System.out.println("IMPORTANT DONE");
		}
		else if(!cameradone)
		{
			cameraInit();
			cameradone = true;
		}
		else
		{
			System.out.println("STARTER DONE");
			super.terminate();
		}
	}
	
	public synchronized boolean importantDone()
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
    	cam.writeBrightness(0);
    	cam.writeExposure(0);
    }
}
