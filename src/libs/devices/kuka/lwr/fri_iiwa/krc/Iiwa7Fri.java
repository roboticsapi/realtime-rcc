package fri;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;


import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation.FRISessionState;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.IExecutionListener;
import com.kuka.roboticsAPI.controllerModel.sunrise.ResumeMode;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseExecutionService;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.executionModel.ExecutionMode;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

public class IiwaFri extends RoboticsAPIApplication {
	private Controller _lbrController;
	private LBR _lbr;
	private final String _hostName = "172.31.1.240";	// IP of the host PC on which the RCC is running
	private final int _controlModePort = 30001;	// Port on which the control mode parameters are sent
	private final int _friPort = 30201;
	private final int PARAMETER_NUMBER = 49;	// number of parameters of the control mode connection
	private final int CONNECTION_CLOSE_NUMBER = 255;

	// Control Mode Protocol UDP objects
	private AbstractMotionControlMode mode;
	private PositionControlMode posControlMode;
	private CartesianImpedanceControlMode cartImpControlMode;
	private CartesianSineImpedanceControlMode cartSineImpControlMode;
	private JointImpedanceControlMode jointImpControlMode;
	private DatagramSocket clientSocket;
	private int currMode = 0;

	private PositionHold posHold;
	private SunriseExecutionService serv;

	private FRISession friSession;
	private FRIJointOverlay ov;
	//private Tool t;

	private InetAddress IPAdress;
	
	private FRIChannelInformation.FRISessionState lastState;
	
	private double[] jointStiffness, jointDamping, cartStiffness, cartDamping, cartSinAmp, cartSinFreq;
	private double[] toolCOM, toolMOI, toolCOC;
	private double toolmass;
	
	private Thread controlModeThread = null;
	
	private boolean doCycleFlag = true;
	private boolean threadStarted = false;
	
	private Runnable _runnable = new Runnable(){

		@Override
		public void run() {
			System.out.println(".. Thread started !");
			threadStarted = false;
			sendUDPMessage("#");	//Heartbeat-message
			System.out.println("Heartbeat sent!");
			serv.resumeExecution(ResumeMode.NoMotion);
			
			// INIT:
			if(!openSocket()){
				System.out.println("Error while opening Socket!");
				return;
			}
			
			// CYCLE:
			while (true) {
				//doCycleFlag = true;
				
				try {
					double d[] = null;
					int lastMode = 0;
					
					if(clientSocket != null && !clientSocket.isClosed()){
						d = toDoubleArray(receiveArrayOverUDP());
						lastMode = currMode;
						currMode = (int) d[0];
					}
					else{
						System.err.println("UDP Socket is null!");
						return;
					}
					
					//System.out.println("ControlModeMessage received! " + currMode);
					
					if(lastMode != currMode){
						if((currMode != 9 || currMode != CONNECTION_CLOSE_NUMBER) && d != null)
							refreshValues(d);
						//showValues();						
						
						switch (currMode) {
						case 0:
							System.out.println("Control Mode is: PositionControlMode");
							mode = posControlMode;
							break;
						case 1:
							System.out.println("Control Mode is: CartesianImpedanceControlMode");
							cartImpControlMode.parametrize(CartDOF.X).setDamping(cartDamping[0]).setStiffness(cartStiffness[0]);
							cartImpControlMode.parametrize(CartDOF.Y).setDamping(cartDamping[1]).setStiffness(cartStiffness[1]);
							cartImpControlMode.parametrize(CartDOF.Z).setDamping(cartDamping[2]).setStiffness(cartStiffness[2]);
							cartImpControlMode.parametrize(CartDOF.A).setDamping(cartDamping[3]).setStiffness(cartStiffness[3]);
							cartImpControlMode.parametrize(CartDOF.B).setDamping(cartDamping[4]).setStiffness(cartStiffness[4]);
							cartImpControlMode.parametrize(CartDOF.C).setDamping(cartDamping[5]).setStiffness(cartStiffness[5]);
							mode = cartImpControlMode;
							break;
						case 2:
							System.out.println("Control Mode is: JointImpedanceControlMode");
							jointImpControlMode.setDamping(jointDamping);
							jointImpControlMode.setStiffness(jointStiffness);
							mode = jointImpControlMode;
							break;
						case 3:
							System.out.println("Control Mode is: CartesianSinImpedanceControlMode");
							cartSineImpControlMode.parametrize(CartDOF.X).setDamping(cartDamping[0]).setStiffness(cartStiffness[0]).setAmplitude(cartSinAmp[0]).setFrequency(cartSinFreq[0]);
							cartSineImpControlMode.parametrize(CartDOF.Y).setDamping(cartDamping[1]).setStiffness(cartStiffness[1]).setAmplitude(cartSinAmp[1]).setFrequency(cartSinFreq[1]);
							cartSineImpControlMode.parametrize(CartDOF.Z).setDamping(cartDamping[2]).setStiffness(cartStiffness[2]).setAmplitude(cartSinAmp[2]).setFrequency(cartSinFreq[2]);
							cartSineImpControlMode.parametrize(CartDOF.A).setDamping(cartDamping[3]).setStiffness(cartStiffness[3]).setAmplitude(cartSinAmp[3]).setFrequency(cartSinFreq[3]);
							cartSineImpControlMode.parametrize(CartDOF.B).setDamping(cartDamping[4]).setStiffness(cartStiffness[4]).setAmplitude(cartSinAmp[4]).setFrequency(cartSinFreq[4]);
							cartSineImpControlMode.parametrize(CartDOF.C).setDamping(cartDamping[5]).setStiffness(cartStiffness[5]).setAmplitude(cartSinAmp[5]).setFrequency(cartSinFreq[5]);
							cartSineImpControlMode.setStayActiveUntilPatternFinished(true);

							mode = cartSineImpControlMode;
							break;
						/*case 5:
							System.out.println("Control Mode Listener exited!");
							clientSocket.close();
							return;*/
						case 9:
							//Heartbeat from RCC
							break;
							
						case CONNECTION_CLOSE_NUMBER:	// close connection
							System.out.println("Closing connection ..");
							
							break;
						default:
							System.out.println("Control Mode is: PositionControlMode");
							mode = posControlMode;
							break;
						}
						
						//change tool data while running, necessary?
						/*if (t == null)
							t = new Tool("tool", new LoadData(toolmass, Transformation.ofRad(
									toolCOM[0],toolCOM[1],toolCOM[2],toolCOM[3],toolCOM[4],toolCOM[5]), Inertia.of(toolMOI[0], toolMOI[1], toolMOI[2])));
						t.detach();
						t.attachTo(_lbr.getFlange());*/
						
						//set COC
						//ObjectFrame of = t.addChildFrame("COC", Transformation.ofRad(0, 0, 0, 0, 0, 0));
						//t.removeFrame(of);
						//t.setDefaultMotionFrame(of);
						
						

						// cancel motion and set new ControlMode
						if(currMode != 9 || currMode != CONNECTION_CLOSE_NUMBER ){
							if (currMode != 3)
								posHold.setMode(mode);
							serv.cancelAll();
							serv.resumeExecution(ResumeMode.NoMotion);
						}
						
						if(currMode == CONNECTION_CLOSE_NUMBER){
							doCycleFlag = false;
							serv.cancelAll();								
							//friSession.close();
						}

						
					}
					
					// send success-reply message over UDP socket
					if(clientSocket != null){
						if(!clientSocket.isClosed()){
							sendUDPMessage(String.valueOf(currMode));
							if(currMode == CONNECTION_CLOSE_NUMBER){
								closeSocket();
								controlModeThread = null;
								currMode = 0;
								
								/*try {
									controlModeThread.join();
								} catch (InterruptedException e) {
									e.printStackTrace();
								}*/
								System.out.println("Thread closed!");
								return;
							}
								
						}
						
					}

				} catch (IOException e) {
					System.err.println("Thread Error: " + e.getMessage()
							+ "\nServerThread exited!");
					closeSocket();
					//return;
				}
			}
		}
		
	};

	/**
	 * Initialize function. Is executed automatically when application is started.
	 */
	@Override
	public void initialize() {
		// get controller, robot and execution service objects
		_lbrController = (Controller) getContext().getControllers().toArray()[0];
		_lbr = (LBR) _lbrController.getDevices().toArray()[0];
		serv = (SunriseExecutionService) _lbrController.getExecutionService();

		// all possible control modes
		posControlMode = new PositionControlMode();
		cartImpControlMode = new CartesianImpedanceControlMode();
		cartSineImpControlMode = new CartesianSineImpedanceControlMode();
		jointImpControlMode = new JointImpedanceControlMode(7);
		mode = posControlMode;

		// configure and start FRI session
		FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _hostName);
		friConfiguration.setSendPeriodMilliSec(4);
		friConfiguration.setPortOnController(_friPort);
		friConfiguration.setPortOnRemote(_friPort);
		friSession = new FRISession(friConfiguration);
		ov = new FRIJointOverlay(friSession);
		
		// current parameters of the motion
		jointStiffness = new double[7];
		jointDamping = new double[7];
		cartStiffness = new double[6];
		cartDamping = new double[6];
		cartSinAmp = new double[6];
		cartSinFreq = new double[6];
		toolCOM = new double[6];
		toolMOI = new double[3];
		toolCOC = new double[6];
		
		// state listener
		lastState = FRISessionState.IDLE;
		IFRISessionListener listener = new IFRISessionListener() {
			@Override
			public void onFRIConnectionQualityChanged(
					FRIChannelInformation friChannelInformation) {
				System.out.println("Connection Quality changed to "
						+ friChannelInformation.getQuality().toString()); 
			}

			@Override
			public void onFRISessionStateChanged( 
					FRIChannelInformation friChannelInformation) {
				FRISessionState currState = friChannelInformation.getFRISessionState();
				FRISessionState idleState = FRIChannelInformation.FRISessionState.IDLE;
				FRISessionState monitoringWaitState = FRIChannelInformation.FRISessionState.MONITORING_WAIT;
				
				if(currState != lastState){
					if(currState != idleState && lastState == idleState){
						//sendUDPMessage("#");	//Heartbeat-message
						//System.out.println("Heartbeat sent!");
					}
					else if(currState == FRISessionState.IDLE){
						System.out.println("Connection to RCC lost or closed!");
						//closeSocket();
						/*if(controlModeThread.isAlive()){
							try {
								controlModeThread.join();
							} catch (InterruptedException e) {
								// TODO Automatisch generierter Erfassungsblock
								e.printStackTrace();
							}
							closeSocket();
						}*/
					}
					else if(currState == monitoringWaitState){
						
					} 
					else if(currState == FRISessionState.COMMANDING_ACTIVE){
							//openSocket(); 
						if(controlModeThread == null && !threadStarted){
							threadStarted = true;
							System.out.println("Starting Thread ..");
							controlModeThread = new Thread(_runnable);
							controlModeThread.start();							
						}
					}
					lastState = currState;
				}
				
			}
		};
		friSession.addFRISessionListener(listener);
		
		// NOT WORKING properly at the moment
		serv.addExecutionListener(new IExecutionListener() {
			
			@Override
			public void onExecutionResumed(Controller controller, ResumeMode mode) {
				// resume FRI connection
				//openSocket();
				if(!controlModeThread.isAlive())
					controlModeThread.start();
				System.out.println("Execution Resumed!");
			}
			
			@Override
			public void onExecutionPaused(Controller controller) {
				// pause FRI connection
				try {
					if(controlModeThread.isAlive())
						controlModeThread.join();
				} catch (InterruptedException e) {
					// TODO Automatisch generierter Erfassungsblock
					e.printStackTrace();
				}
				closeSocket();
				System.out.println("Execution Paused!");
			}
			
			@Override
			public void onExecutionModeChanged(Controller controller,
					ExecutionMode newMode) {				
				
			}
		});		
		
	}

	/**
	 * Overwritten run function, which is executed automatically when application is started.
	 */
	@Override
	public void run() {
		
		//set Flange
		//Tool flange = getApplicationData().createFromTemplate("LeererFlansch");
		//flange.attachTo(_lbr.getFlange());

		// move to start position
		moveHome();

		// Control Mode Listen Thread
		//controlModeThread = new Thread(_runnable);
		//controlModeThread.start();
		

		// endless cycle for multiple connections from RCC
		while(true){
			
			// wait until FRI session is ready to switch to command mode
			try {
				
				System.out.println("Waiting for RCC FRI connection ..");
				friSession.await(Long.MAX_VALUE, TimeUnit.SECONDS);
				
				/*if(!controlModeThread.isAlive())
					controlModeThread.start();
				else
					openSocket();*/
				
				posHold = new PositionHold(mode, -1, TimeUnit.SECONDS);

				// ##### FRI MotionOverlay MOVE-CYCLE #####
				while (friSession != null /*&& doCycleFlag*/) {
					if(mode != cartSineImpControlMode){
						// Position hold with overlay in modes: Position-, SinusImpedance-, JointImpedance
						System.out.println("Overlay control ready!");
						
						// ### OVERLAY EXECUTION ###
						_lbr.move(posHold.addMotionOverlay(ov));				
					}
					else{
						// Cartesian Sinus Impedance Mode not possible in combination with move 
						// -> no overlay, just oscillation
						posHold = new PositionHold(mode, -1, TimeUnit.SECONDS);
						_lbr.move(posHold.setMode(mode));
					}
				}
				
			// catch 10 secs await Timeout
			} catch (final TimeoutException e) {
				System.err.println("Timeout occured - FRISessionstate: "
						+ friSession.getFRIChannelInformation());
				closeSocket();
				
			// catch bad connection quality or connection closed (CK_COMPOUND_RETURN_ERROR / WRONG_SESSION_STATE_MONITORING_WAIT)
			}catch (final CommandInvalidException e){
				if(doCycleFlag){
					System.err.println("CommandInvalidException - Connection closed!\n"+e.getMessage()+"\nConnection Quality too bad or connection to RCC closed!");
					closeSocket();
				}
				else{
					doCycleFlag = true;
				}
			}
		}
		

		// close connection and application in case of an error
		//friSession.close();
	}
		
	/**
	 * function rounds a value depending on given values
	 * @param value - value which should be rounded
	 * @param places - amount of digits to which the value should be rounded
	 * @return rounded double value
	 */
	private double roundDouble(double value, int places) {
	    if (places < 0) throw new IllegalArgumentException();

	    long factor = (long) Math.pow(10, places);
	    value = value * factor;
	    long tmp = Math.round(value);
	    return (double) tmp / factor;
	}
	
	/**
	 * function refreshes current parameters
	 * @param array - received double array
	 * @return if operation is successful
	 */
	private boolean refreshValues(double[] array){
		if(array.length < PARAMETER_NUMBER){
			System.err.println("too less values for new control Mode!");
			return false;
		}
		else{
			for(int i = 1; i < array.length -1; ++i){
				if(i < 8){
					jointStiffness[i - 1] = array[i];
				}
				else if(i < 15){
					jointDamping[i - 8] = array[i];
				}
				else if(i < 21){
					cartStiffness[i - 15] = array[i];
				}
				else if (i < 27){
					cartDamping[i - 21] = array[i];
				}
				else if(i < 33){
					cartSinAmp[i - 27] = array[i];
				}
				else if (i < 39){
					cartSinFreq[i - 33] = array[i];
				}
				else if(i < 40){
					//System.out.println("ToolMass: "+d[27]);
					toolmass = array[i];
				}
				else if(i < 46){
					toolCOM[i - 40] = array[i];
					//System.out.println("ToolCOM ["+(i-28)+"] : "+d[i]);
				}
				else{
					toolMOI[i - 46] = array[i];
				}
			}

			return true;
		}		
	}
	
	/**
	 * function shows some current values - for debugging purposes
	 */
	@SuppressWarnings(value = { "unused" })
	private void showValues(){
		for(double v:jointStiffness)
			System.out.println("jointStiffness Value: "+v);
		for(double v:jointDamping)
			System.out.println("jointDamping Value: "+v);
		for(double v:cartStiffness)
			System.out.println("cartStiffness Value: "+v);
		for(double v:cartDamping)
			System.out.println("cartDamping Value: "+v);

	}
	
	/**
	 * Function converts the a byte array to an array of double values.
	 * It is used to process received data from the control mode connection. 
	 * @param byteArray - byte wise array of values
	 * @return converted double array 
	 */
	private double[] toDoubleArray(byte[] byteArray){
	    int times = Double.SIZE / Byte.SIZE;
	    double[] doubles = new double[byteArray.length / times];
	    for(int i=0;i<doubles.length;i++){
	        doubles[i] = ByteBuffer.wrap(byteArray, i*times, times).getDouble();
		//swap all bits of received double, because its in c++ double format
	    long l1 = Double.doubleToLongBits(doubles[i]);
		long l2 = Long.reverseBytes(l1);
		doubles[i] = roundDouble(Double.longBitsToDouble(l2),7);
	    }
	    return doubles;
	}
	
	/**
	 * Simple home move
	 */
	private void moveHome(){
		serv.setExecutionMode(ExecutionMode.Continuous);
		JointPosition homePos = new JointPosition(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0);
		//JointPosition homePos2 = new JointPosition(.0, .0, .0, .0, .0, .0, .0);
		_lbr.move(ptp(homePos));
		System.out.println("Robot was moved to the initial configuration.");
	}
	
	/**
	 * Opens the control mode connection socket.
	 * @return if opening was successful
	 */
	private boolean openSocket(){
		if(clientSocket != null){
			if(!clientSocket.isClosed()){
				return true;
			}
		}
		else{
			System.out.println("Opening Socket ..");
			try {
				clientSocket = new DatagramSocket(_controlModePort);
				
			} catch (SocketException e2) {
				System.err.println("DatagrammSocket Init Error: "
						+ e2.getMessage());
				return false;
			}
			try {
				IPAdress = InetAddress.getByName(_hostName);
			} catch (UnknownHostException e2) {
				System.err.println("InetAddress Init Error: "
						+ e2.getMessage());
				return false;
			}
			System.out.println(".. Socket opened!");
		}
		
		return true;
	}
	
	/**
	 * Receives data from the control mode connection.
	 * @return byte array which was received
	 * @throws IOException
	 */
	private byte[] receiveArrayOverUDP() throws IOException{
		byte[] receiveData = new byte[PARAMETER_NUMBER * 8];
		DatagramPacket receivePacket;
		
		receivePacket = new DatagramPacket(receiveData,
				receiveData.length);
		clientSocket.receive(receivePacket);
		
		return receivePacket.getData();
	}
	
	/**
	 * Sends message over the control mode connection.
	 * @param m - String which should be sent
	 * @return if function was successful
	 */
	private boolean sendUDPMessage(String m){
		if(clientSocket == null){
			openSocket();
		}
		byte[] sendData = new byte[1];
		DatagramPacket sendPacket;

		sendData = m.getBytes();
		sendPacket = new DatagramPacket(sendData, sendData.length,
				IPAdress, _controlModePort);
		try {
			clientSocket.send(sendPacket);
			//System.out.println("send Message "+m+" established");
			return true;
		} catch (IOException e1) {
			e1.printStackTrace();
			System.out.println("message sending FAILED!");
			return false;
		}
		
	}
	
	/**
	 * Closes the control mode connection.
	 */
	private void closeSocket(){
		if(clientSocket != null && !clientSocket.isClosed()){
			clientSocket.close();
			clientSocket = null;
			System.out.println("ClientSocket closed !");
		}
		else{
			//System.out.println("ClientSocket closed already !");
		}
	}

	/**
	 * MAIN function - creates an instance of the application and runs it.
	 * @param args
	 */
	public static void main(String[] args) {
		final IiwaFri app = new IiwaFri();
		app.runApplication();
	}
}
