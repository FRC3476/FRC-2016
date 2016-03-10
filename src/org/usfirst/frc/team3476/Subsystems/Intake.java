package org.usfirst.frc.team3476.Subsystems;


import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.ManualHandler;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.Control.PIDCANTalonEncoderWrapper;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class Intake implements Subsystem
{
	private double SUCKMOTORSPEED, LOADMOTORSPEED, ddTime, INTAKE1DIR, INTAKE2DIR, DDCONTROLP,
	DDCONTROLI, DDCONTROLD, DDCONTROLDEAD, DDCONTROLOUTPUTHIGH, DDCONTROLOUTPUTLOW, DDCHANNEL,
	INTAKEHOMINGCURRENT, INTAKEHOMINGSPEED, DEFAULTDROPDOWNPOS;
	
	final String[] autoCommands = {"intake", "dropdown"};
	final String[] constants = {"SUCKMOTORSPEED", "LOADMOTORSPEED", "FORWARDISDOWN", "INTAKE1DIR",
			"INTAKE2DIR", "DDCONTROLP", "DDCONTROLI", "DDCONTROLD", "DDCONTROLDEAD",
			"DDCONTROLOUTPUTHIGH", "DDCONTROLOUTPUTLOW", "DDCHANNEL", "INTAKEHOMINGCURRENT",
			"INTAKEHOMINGSPEED", "DEFAULTDROPDOWNPOS"};
	
	private boolean done, FORWARDISDOWN, started;
	
	final long MANUALTIMEOUT = 50;
	
	private SpeedController intake1, intake2;
	
	private CANTalon ddmotor;
	private PIDController ddController;
	private PIDCANTalonEncoderWrapper ddwrapper;
	private PowerDistributionPanel pdPanel;
	
	private ManualHandler ddManual, intakeManual;
	
	private SubsystemTask task;
	private Thread ddThread;
	private double ddPosition, lastposition;
	private boolean stopdd;
	private boolean homed;

	public Intake(SpeedController intake1in, SpeedController intake2in, CANTalon ddmotorin, PowerDistributionPanel pdPanelin)
	{
		pdPanel = pdPanelin;
		
		//Intake
		intake1 = intake1in;
		intake2 = intake2in;
		ddmotor = ddmotorin;
		
		ddwrapper = new PIDCANTalonEncoderWrapper(ddmotor, 1);
		
		//Dropdown
		ddPosition = 0;
		lastposition = 0;
		ddController = new PIDController(0, 0, 0, ddwrapper, ddmotor);
		ddController.disable();
		ddController.setOutputRange(0, 0);
		ddController.setToleranceBuffer(6);
		stopdd = false;
		homed  = false;
		
		//Manuals
		ddManual = new ManualHandler(MANUALTIMEOUT);
		intakeManual = new ManualHandler(MANUALTIMEOUT);
		
		started = true;
		done = true;
		
		
		//Thread
		task = new SubsystemTask(this, 10);
		ddThread = new Thread(task, "ddThread");
		ddThread.start();
	}
	
	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}

	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		done = false;
		started = false;
		if(command.equalsIgnoreCase("intake"))
		{
			System.out.println("intaking " + params[0] + " at " + params[1] + "percent");
			//Direction(sign(possibly 0)), percent speed, constant to invert if necessary and make timing correct
			intake1.set(params[0]*params[1]*SUCKMOTORSPEED/100);
			intake2.set(params[0]*params[1]*LOADMOTORSPEED/100);
			done = true;
		}
		else if(command.equalsIgnoreCase("dropdown"))
		{
			switch((int)params[0])
			{
			}
			ddTime = params[1];
		}
	}

	@Override
	public synchronized boolean isAutoDone()
	{
		return done;
	}

	@Override
	public String[] getConstantRequest()
	{
		return constants;
	}

	@Override
	public synchronized void returnConstantRequest(double[] constantsin)
	{
		double 	prevDDdead = DDCONTROLDEAD,
				prevDDp = DDCONTROLP,
				prevDDi = DDCONTROLI,
				prevDDd = DDCONTROLD,
				prevDDouthigh = DDCONTROLOUTPUTHIGH,
				prevDDoutlow = DDCONTROLOUTPUTLOW;
		
		int i = 0;
		SUCKMOTORSPEED = constantsin[i];
		i++;//1
		LOADMOTORSPEED = constantsin[i];
		i++;//2
		FORWARDISDOWN = constantsin[i] == 1;
		i++;//3
		INTAKE1DIR = constantsin[i];
		i++;//4
		INTAKE2DIR = constantsin[i];
		i++;//5
		DDCONTROLP = constantsin[i];
		i++;//6
		DDCONTROLI = constantsin[i];
		i++;//7
		DDCONTROLD = constantsin[i];
		i++;//8
		DDCONTROLDEAD = constantsin[i];
		i++;//9
		DDCONTROLOUTPUTHIGH = constantsin[i];
		i++;//10
		DDCONTROLOUTPUTLOW = constantsin[i];
		i++;//11
		DDCHANNEL = constantsin[i];
		i++;//12
		INTAKEHOMINGCURRENT = constantsin[i];
		i++;//13
		INTAKEHOMINGSPEED = constantsin[i];
		i++;//14
		DEFAULTDROPDOWNPOS = constantsin[i];
		
		if(	prevDDp != DDCONTROLP ||
			prevDDi != DDCONTROLI || prevDDd != DDCONTROLD ||
			prevDDdead != DDCONTROLDEAD || prevDDouthigh != DDCONTROLOUTPUTHIGH ||
			prevDDoutlow != DDCONTROLOUTPUTLOW)//different or null
		{
			String print = "Different constants: ";
			print += prevDDp != DDCONTROLP ? "DDCONTROLP ": "";
			print += prevDDi != DDCONTROLI ? "DDCONTROLI ": "";
			print += prevDDd != DDCONTROLD ? "DDCONTROLD ": "";
			print += prevDDdead != DDCONTROLDEAD ? "DDCONTROLDEAD ": "";
			print += prevDDouthigh != DDCONTROLOUTPUTHIGH ? "DDCONTROLOUTPUTHIGH ": "";
			print += prevDDoutlow != DDCONTROLOUTPUTLOW ? "DDCONTROLOUTPUTLOW": "";
			System.out.println(print);
			
			ddController.setPID(DDCONTROLP, DDCONTROLI, DDCONTROLD);
			ddController.setAbsoluteTolerance(DDCONTROLDEAD);
			ddController.setOutputRange(DDCONTROLOUTPUTLOW, DDCONTROLOUTPUTHIGH);
		}
	}

	@Override
	public synchronized void update()
	{
		//======================
		//========Intake========
		//======================
		if(intakeManual.isTimeUp())//no longer manual control - do tings
		{
			
		}
		else
		{
			//manual control
		}
		
		//======================
		//=======Dropdown=======
		//======================
		if(ddManual.isTimeUp())//no longer manual control - do tings
		{
			if(homed)
			{
				if(!stopdd)
				{
					if(!ddController.isEnabled())
					{
						ddController.enable();
					}
					
					if(ddPosition != lastposition)
					{
						ddController.setSetpoint(ddPosition);
					}
					lastposition = ddPosition;
				}
				else
				{
					if(ddController.isEnabled())
					{
						ddController.disable();
					}
				}
			}
			else//homing
			{
				if(ddController.isEnabled())
				{
					ddController.disable();
				}
				
				if(pdPanel.getCurrent((int)DDCHANNEL) > INTAKEHOMINGCURRENT)
				{
					ddmotor.set(0);
					ddwrapper.reset();
					moveDropdown(DEFAULTDROPDOWNPOS);
					homed = true;
				}
				else
				{
					ddmotor.set(INTAKEHOMINGSPEED);
				}
			}
		}
		else//manual control
		{
			if(ddController.isEnabled())
			{
				ddController.disable();
			}
		}
		
		
		/*if(!started && !done)
		{
			ddTimer.reset();
			ddTimer.start();
			setIntakeMovement(curDir);
			started = true;
		}
		else if(started && !done)
		{
			done = ddTimer.hasPeriodPassed(ddTime);
			if(done)
			{
				ddTimer.stop();
				setIntakeMovement(DDdir.STOP);
			}
		}*/
	}
	
	/*public void setIntakeMovement(DDdir dir)
	{
		Value forward = Relay.Value.kForward;
		Value reverse = Relay.Value.kReverse;
		
		switch(dir)
		{
			case UP:
				ddmotor.set(FORWARDISDOWN ? reverse : forward);
				break;
			case DOWN:
				ddmotor.set(FORWARDISDOWN ? forward : reverse);
				break;
			case STOP:
				ddmotor.set(Relay.Value.kOff);
				break;
		}
	}*/
	
	public double getDDSet()
	{
		return ddController.getSetpoint();
	}
	
	public void stopDD()
	{
		stopdd = true;
	}
	
	public boolean intakeRunning()
	{
		return intake1.get() != 0 || intake2.get() != 0;
	}
	
	public void printPID()
	{
		String print = "Different constants: ";
		print += "DDCONTROLP " + DDCONTROLP + " ";
		print += "DDCONTROLI " + DDCONTROLI + " ";
		print += "DDCONTROLD " + DDCONTROLD + " ";
		print += "DDCONTROLDEAD " + DDCONTROLDEAD;
		System.out.println(print);
	}
	
	public PIDCANTalonEncoderWrapper getIntakeEncoder()
	{
		return ddwrapper;
	}
	
	public void moveDropdown(double position)
	{
		stopdd = false;
		ddPosition = position;
	}
	
	public void manualIntake(double speed)
	{
		intakeManual.poke();
		setIntake(speed);
	}
	
	private void setIntake(double speed)
	{
		intake1.set(speed*SUCKMOTORSPEED*INTAKE1DIR);
		intake2.set(speed*SUCKMOTORSPEED*INTAKE2DIR);
	}
	
	public void manualDropdown(double move)
	{
		ddManual.poke();
		ddmotor.set(move);
	}
	
	public String toString()
	{
		return "Intake";
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
			ddThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}

	@Override
	public void end()
	{
		//setIntakeMovement(DDdir.STOP);
		intake1.set(0);
		intake2.set(0);
	}
	
	@Override
	public boolean threadsActive()
	{
		return task.isActive();
	}
}
