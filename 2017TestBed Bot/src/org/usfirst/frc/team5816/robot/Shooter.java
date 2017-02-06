package org.usfirst.frc.team5816.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

public class Shooter implements Runnable{
	private final int PORT = 4144;
	private ServerSocket server;
	private Socket client;
	private PrintWriter output;
	private BufferedReader input;
	
	private Double angle = 0d;
	private Boolean isVisible = false;
	
	public Shooter(){
		Thread core = new Thread(this);
		core.start();
	}
	
	private static void sleep(long time){
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public void run(){
		
		try {
			this.server = new ServerSocket(this.PORT);
		} catch (IOException e) {
			e.printStackTrace();
			return;
		}
		while(true){
			try {
				this.acceptClient();
				break;
			} catch (IOException e) {
				this.removeClient();
				e.printStackTrace();
			}
		}
		System.out.println("Yo Yo Yo");
		Shooter.sleep(2000);
		long pingTime = -1;
		try {
			 pingTime = this.ping();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		for(int index = 0; index<200; index++){
			System.out.equals(pingTime);
		}
		
		
		while(true){
			String data = null;
			try {
				data = this.reciveData();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			if(data!=null){
				System.out.println(data);
				this.parseData(data);
			}
			
		}
		
	}
	
	public void acceptClient() throws IOException{
		this.client = this.server.accept();
		this.input = new BufferedReader(new InputStreamReader(this.client.getInputStream()));
		this.output = new PrintWriter(this.client.getOutputStream());
	}
	
	public void removeClient(){
		this.client = null;
		this.input = null;
		this.output = null;
	}
	
	public void sendData(String data){
		this.output.println(data);
	}
	
	public String reciveData() throws IOException{
		return this.input.readLine();
	}
	
	private void parseData(String data){
		String[] tokens = data.split(":");
		Double arg0 = null;
		try{
			arg0 = Double.parseDouble(tokens[1]);
		}catch(Exception e){
			e.printStackTrace();
			return;
		}
		switch(tokens[0]){
		case "ANGLE":{
			this.angle = arg0;
			break;
		}
		case "VISIBLE":{
			this.isVisible = (arg0==1);
			break;
		}
		}
		
	}
	
	public double getAngle(){
		return this.angle;
	}
	
	public void setAngle(double angle){
		this.angle = angle;
	}

	public boolean isVisible() {
		return isVisible;
	}

	public void setIsVisible(boolean isVisible) {
		this.isVisible = isVisible;
	}
	public long ping() throws IOException{
		Long startTime = System.nanoTime();
		this.sendData("PING");
		while(this.reciveData()==null){}
		return (System.nanoTime() - startTime)/1000000;
	}

}
