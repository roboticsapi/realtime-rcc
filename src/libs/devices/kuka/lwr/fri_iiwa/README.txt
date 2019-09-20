The following describes the usage of the FRI components, to establish a connection between the RCC and the KUKA Iiwa Robot

Requirements:
 - KUKA Iiwa 7 or 14 Robot in automation (AUT) and safe operating mode
 - IiwaFri application on the robot control 
 - Host PC with realtime OS (e.g. Ubuntu with xenomai kernel)
 - Realtime capable network card on PC
 - Direct LAN connection between the Iiwa Robot and the Host PC (use realtime network slot!)
 - RCC with correct adjusted device.xml and extension.xml (IiwaFri Device with correct IPs and Ports)
 
 
Steps to establish connection:
 - start robot and host PC ;) 
 	-> on first Iiwa start you have to press and release an external stop one time to accomplish the sufficient robot state
 - start rtnet kernel module on host PC (automatically with a shell script if available (sudo ./startrtnet.sh) 
 											- or manually with rmmod and insmod)
 - check rt connection with rtping (sudo /usr/local/rtnet/sbin/rtping <IP>)
 - start IiwaFri application on the robot (ATTENTION, robot moves to initial position!)
 - on host PC go to your RCC/build location (e.g. cd workspace/OrocosRCC/build) and start RCC (sudo ./RealtimeRCC)
 - connection should be established - check Errors and Warnings on RCC and KUKA Panel log
 - now you can start a Java RoboticsAPI Application to control the Iiwa Robot 
 	(use same robot name in the config.xml of the Java application as in device.xml of the RCC)
 
 Possible Issues and Errors:
  - RCC: Iiwa Device could not be created 
  	-> make sure all necessary Devices have been added to the extensions.xml
  	-> also make sure the settings in the device.xml fit to the Device
  - RCC: connection issues (e.g. fri connection over port xy could not be established)
  	-> check the connection to the robot -> use rtping
  	-> adjust the parameters in the device.xml
  	-> check LAN connection
  - During Motion: motion stops and Iiwa application shows errors
  	-> restart Iiwa application
  
