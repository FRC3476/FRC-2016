package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.Control.PIDDashdataWrapper;
import org.usfirst.frc.team3476.Utility.Control.TakeBackHalf;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class Shooter implements Subsystem
{
	private final String[] autoCommands = {"shooter", "aim", "flywheel", "fire", "loader"};
	private final String[] constants = {"SHOOTEROUTPUTRANGEHIGH", "SHOOTEROUTPUTRANGELOW", "SHOOTERIGAIN", "FLY1DIR", "FLY2DIR", "FLYWHEELDEAD", "FLYWHEELMAXSPEED", "TURRETP", "TURRETI", "TURRETD"};
	private final double RPMTORPS = 60;
	
	enum Aim{UP, DOWN}
	enum Load{IN, OUT}
	
	private double SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW, SHOOTERIGAIN, FLYWHEELDEAD, FLYWHEELMAXSPEED, TURRETP, TURRETI, TURRETD;
	private double[] FLYDIRS;
	private boolean AIMUPPOWERED, flyDone, loadDone, firing, firingLast, pass1, turretdone;
	private SpeedController fly1, fly2, turret;
	private TakeBackHalf control;
	private Counter tach;
	private Timer shootingTimer;
	
	private PIDDashdataWrapper vision;
	private PIDController turretcontrol;
	
	private SubsystemTask task;
	private Thread flyThread;
	
	public Shooter(SpeedController fly1in, SpeedController fly2in, SpeedController turret, Solenoid aimin, Solenoid loaderin, Counter tachin)
	{
		vision = new PIDDashdataWrapper("Camera Data");
		turretcontrol = new PIDController(TURRETP, TURRETI, TURRETD, vision, turret);
		turretcontrol.enable();
		turretdone = true;
		
		fly1 = fly1in;
		fly2 = fly2in;
		flyDone = true;
		loadDone = true;
		tach = tachin;
		firing = false;
		shootingTimer = new Timer();
		FLYDIRS = new double[2];
		
		task = new SubsystemTask(this);
		flyThread = new Thread(task, "flyThread");
		flyThread.start();
	}
	
	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}
	
	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		switch(command)
		{
			case "shooter":
				flyDone = false;
				
				control.setSetpoint(params[1]);
				aim(params[0] == 1 ? Aim.UP : Aim.DOWN);
				break;
			case "aim":
				aim(params[0] == 1 ? Aim.UP : Aim.DOWN);
				break;
			case "flywheel":
				flyDone = false;
				
				control.setSetpoint(params[0]);
				break;
			case "fire":
				loadDone = false;
				
				startFire();
				break;
			case "loader":
				loader(params[0] == 1 ? Load.OUT : Load.IN);
				break;
		}
	}
	
	@Override
	public synchronized boolean isAutoDone()
	{
		return flyDone && loadDone;
	}
	
	@Override
	public String[] getConstantRequest()//Request all needed constants
	{
		return constants;
	}
	
	@Override
	public synchronized void returnConstantRequest(double[] constantsin)//Get all needed constants
	{
		int i = 0;
		AIMUPPOWERED = constantsin[i] == 1 ? true : false;
		i++;//1
		SHOOTEROUTPUTRANGEHIGH = constantsin[i];
		i++;//2
		SHOOTEROUTPUTRANGELOW = constantsin[i];
		i++;//3
		SHOOTERIGAIN = constantsin[i];
		i++;//4
		FLYDIRS[i - 4] = constantsin[i];
		i++;//5
		FLYDIRS[i - 4] = constantsin[i];
		i++;//6
		FLYDIRS[i - 4] = constantsin[i];
		i++;//7
		FLYDIRS[i - 4] = constantsin[i];
		i++;//8
		GRABFRISBEETIME = constantsin[i];
		i++;//9
		SHOOTFRISBEETIME = constantsin[i];
		i++;//10
		FLYWHEELDEAD = constantsin[i];
		i++;//11
		FLYWHEELMAXSPEED = constantsin[i];
		
		control = new TakeBackHalf(new double[]{SHOOTEROUTPUTRANGEHIGH, SHOOTEROUTPUTRANGELOW}, SHOOTERIGAIN, FLYWHEELMAXSPEED);
		control.setSetpoint(0);
		startThreads();
	}

	@Override
	public synchronized void update()//Flywheel control loop
	{
		//Take back half control
		double output = 0;
		/*double process = tach.getRate()*RPMTORPS;//Get rps > to rpm
		if(control == null)
		{
			throw new NullPointerException("No TakeBackHalf controller in Subsystem \"" + this +  "\" - constants not returned");
		}
		else
		{
			output = control.output(process);
		}*/
		output = control.getSetpoint() > 0 ? 1 : 0;
		fly1.set(output*FLYDIRS[0]);
		fly2.set(output*FLYDIRS[1]);
		
		//Turret update
		if(!turretdone)
		{
			
		}
		
		//Shooter update
		if(firing && !firingLast)//Starting firing sequence
		{
			shootingTimer.reset();
			shootingTimer.start();
			loader(Load.OUT);
			pass1 = false;
		}
		else if(firing && firingLast)//Update firing sequence
		{
			if(shootingTimer.get() > GRABFRISBEETIME && !pass1)
		    {
				shootingTimer.reset();
		    	loader(Load.IN);
		    	pass1 = true;
		    }
			if(shootingTimer.get() > SHOOTFRISBEETIME && pass1)
		    {
		    	shootingTimer.stop();
		    	shootingTimer.reset();
		    	firing = false;
		    }
		}
		firingLast = firing;
		
		//Check if we're done here 
		//TODO: Decide if we need to wait for the flywheel needs to be in the deadzone for multiple iterations
		if(true /*Math.abs(control.getSetpoint() - process) < FLYWHEELDEAD*/) flyDone = true;
		if(!firing) loadDone = true;
	}
	
	public synchronized void startFire()
	{
		firing = true;
	}
	
	public synchronized boolean isFiring()
	{
		return firing;
	}
	
	public synchronized void aim(Aim dir)
	{
		switch(dir)
		{
			case UP:
				aim.set(AIMUPPOWERED ? true : false);
				break;
			case DOWN:
				aim.set(AIMUPPOWERED ? false : true);
				break;
		}
	}
	
	public synchronized void loader(Load dir)
	{
		switch(dir)
		{
			case IN:
				loader.set(false);
				break;
			case OUT:
				loader.set(true);
				break;
		}
	}
	
	public synchronized Load getLoader()
	{
		return loader.get() ? Load.OUT : Load.IN;
	}
	
	public String toString()
	{
		return "Shooter";
	}
	
	@Override
	public void stopThreads()
	{
		task.hold();
	}
	
	@Override
	public void startThreads()
	{
		task.resume();
	}
	
	public void terminateThreads()
	{
		task.terminate();
		try
		{
			flyThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}
	
	public void end()
	{
		flyDone = false;
		aim(Aim.DOWN);
		loader(Load.IN);
		control.setSetpoint(0);
	}
}
