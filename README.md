Realtime RCC (v2)
=================

This repository contains the Realtime RCC v2, a real-time capable robot control core to be used 
in conjunction with the Robotics API v2. It is based on OROCOS (http://orocos.org) framework and 
communicates with hardware devices via their respective buses and protocols.

To provide real-time guarantees, the Realtime RCC has to run on a real-time capable operating system 
(preferredly Linux with Xenomai extensions), however the implementation also works on standard linux 
for less challenging use cases. 


Background: Robotics API - "Robotics as an API in a modern programming language"
--------------------------------------------------------------------------------

With the Robotics API, you can program complex, real-time critical tasks for your (industrial) robots in Java. 
A convenient, flexible programming model enables you to develop sophisticated applications easy and fast. 
The innovative software architecture of the Robotics API Development Platform ensures reliable execution 
of robotic applications and guarantees exact timing of critical operations.

Using Java as a robot language instead of vendor-specific languages has various advantages. 
You will profit from a modern, object-oriented programming language, hundreds of existing libraries and 
elaborate Integrated Development Environments. Development of the Java ecosystem is actively driven by a 
large community, ensuring that the language will evolve in the future.

The Robotics API Development Platform consists of two separate architectural tiers: Applications are running 
on top of the Java-based Robotics API, while real-time critical device control is done by a Robot Control Core (RCC). 
The RCC has to run on a real-time capable operating system which can communicate with all hardware devices via 
the respective buses and protocols. Robotics API applications can run on any Java-capable operating system on any 
hardware, as long as it can communicate with the RCC via network. For simulation purposes, and to control devices 
that do not require hard real-time control, a Java implementation of a Robot Control Core is bundled with the Robotics API. 

For more details, visit https://www.roboticsapi.org/


Background: Robotics API v2
---------------------------

Compared to Robotics API v1, this version uses a more flexible execution model on the Robot Control Core.
Additionally, it better handles the requirements posed by mobile robots (e.g., distribution, uncertainty) and includes 
a simple simulation engine. Using the simulation engine, multiple Java robot control core instances can control a shared 
simulated world, allowing to realistically test applications for distributed robots in simulation on a single computer. 


References
----------

An overview of the concepts used in the Realtime RCC can be found in the following articles: 

> Interfacing Industrial Robots using Realtime Primitives   
> Michael Vistein, Andreas Angerer, Alwin Hoffmann, Andreas Schierl, Wolfgang Reif  
> 2010 IEEE International Conference on Automation and Logistics (ICAL), August 16 - 20, 2010, Hong Kong  

> Robotics API: Object-Oriented Software Development for Industrial Robots  
> Andreas Angerer, Alwin Hoffmann, Andreas Schierl, Michael Vistein and Wolfgang Reif  
> Journal of Software Engineering for Robotics, http://joser.unibg.it/index.php/joser/article/view/53


More details can be found in the corresponding dissertation thesis:

> Embedding Real-Time Critical Robotics Applications in an Object-Oriented Language  
> Michael Vistein  
> https://opus.bibliothek.uni-augsburg.de/opus4/3271


License
-------

The Realtime RCC is licensed under the Mozilla Public License (MPL) v2.
