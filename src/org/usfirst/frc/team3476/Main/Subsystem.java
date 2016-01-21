package org.usfirst.frc.team3476.Main;

public interface Subsystem
{
	String[] getAutoCommands();
	
	void doAuto(double[] params, String command);
	
	boolean isAutoDone();
	
	String[] getConstantRequest();
	
	void returnConstantRequest(double[] constantsin);
	
	void update();
	
	void stopThreads();
	
	void startThreads();
	
	void terminateThreads();
	
	void end();
}
