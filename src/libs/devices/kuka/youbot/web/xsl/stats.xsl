<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../../layout.xsl"/>

	<xsl:template match="/controller">
		<div id="navi"><a href="../">youBot Controllers</a></div>
		
		<h2>youBot Statistics <xsl:value-of select="@name" /></h2>
		
		<!-- <p><a href="stats">Communication Statistics</a></p>-->
		<p><a href="parameters">Parameters</a></p>
 		<h2>Axis Parameters</h2>
 		<tr><td></td><th>Axis 1</th><th>Axis 2</th><th>Axis 3</th><th>Axis 4</th><th>Axis 5</th></tr>
		<tr><th>Firmware</th><td><xsl:value-of select="firmwarejoints/@j1" /></td><td><xsl:value-of select="firmwarejoints/@j2" /></td><td><xsl:value-of select="firmwarejoints/@j3" /></td><td><xsl:value-of select="firmwarejoints/@j4" /></td><td><xsl:value-of select="firmwarejoints/@j5" /></td></tr>
		<tr><th>Actual Voltage</th><td><xsl:value-of select="p151/@j1" /></td><td><xsl:value-of select="p151/@j2" /></td><td><xsl:value-of select="p151/@j3" /></td><td><xsl:value-of select="p151/@j4" /></td><td><xsl:value-of select="p151/@j5" /></td></tr>
		<tr><th>Actual driver temperature</th><td><xsl:value-of select="p152/@j1" /></td><td><xsl:value-of select="p152/@j2" /></td><td><xsl:value-of select="p152/@j3" /></td><td><xsl:value-of select="p152/@j4" /></td><td><xsl:value-of select="p152/@j5" /></td></tr>
		
		<h2>Motor/Module Settings</h2>
 		<tr><td></td><th>Axis 1</th><th>Axis 2</th><th>Axis 3</th><th>Axis 4</th><th>Axis 5</th></tr>
		<tr><th>Number of Motor Poles</th><td><xsl:value-of select="p253/@j1" /></td><td><xsl:value-of select="p253/@j2" /></td><td><xsl:value-of select="p253/@j3" /></td><td><xsl:value-of select="p253/@j4" /></td><td><xsl:value-of select="p253/@j5" /></td></tr>
		<tr><th>BEMF constant</th><td><xsl:value-of select="p239/@j1" /></td><td><xsl:value-of select="p239/@j2" /></td><td><xsl:value-of select="p239/@j3" /></td><td><xsl:value-of select="p239/@j4" /></td><td><xsl:value-of select="p239/@j5" /></td></tr>
		<tr><th>Motor coil resistance</th><td><xsl:value-of select="p240/@j1" /></td><td><xsl:value-of select="p240/@j2" /></td><td><xsl:value-of select="p240/@j3" /></td><td><xsl:value-of select="p240/@j4" /></td><td><xsl:value-of select="p240/@j5" /></td></tr>
		<tr><th>Mass inertia constant</th><td><xsl:value-of select="p238/@j1" /></td><td><xsl:value-of select="p238/@j2" /></td><td><xsl:value-of select="p238/@j3" /></td><td><xsl:value-of select="p238/@j4" /></td><td><xsl:value-of select="p238/@j5" /></td></tr>
		<tr><th>PWM-Hysteresis</th><td><xsl:value-of select="p136/@j1" /></td><td><xsl:value-of select="p136/@j2" /></td><td><xsl:value-of select="p136/@j3" /></td><td><xsl:value-of select="p136/@j4" /></td><td><xsl:value-of select="p136/@j5" /></td></tr>
		<tr><th>Thermal winding time constant</th><td><xsl:value-of select="p25/@j1" /></td><td><xsl:value-of select="p25/@j2" /></td><td><xsl:value-of select="p25/@j3" /></td><td><xsl:value-of select="p25/@j4" /></td><td><xsl:value-of select="p25/@j5" /></td></tr>
		<tr><th>I²t limit</th><td><xsl:value-of select="p26/@j1" /></td><td><xsl:value-of select="p26/@j2" /></td><td><xsl:value-of select="p26/@j3" /></td><td><xsl:value-of select="p26/@j4" /></td><td><xsl:value-of select="p26/@j5" /></td></tr>
		<tr><th>I²t sum</th><td><xsl:value-of select="p27/@j1" /></td><td><xsl:value-of select="p27/@j2" /></td><td><xsl:value-of select="p27/@j3" /></td><td><xsl:value-of select="p27/@j4" /></td><td><xsl:value-of select="p27/@j5" /></td></tr>
		<tr><th>I²t exceed counter</th><td><xsl:value-of select="p28/@j1" /></td><td><xsl:value-of select="p28/@j2" /></td><td><xsl:value-of select="p28/@j3" /></td><td><xsl:value-of select="p28/@j4" /></td><td><xsl:value-of select="p28/@j5" /></td></tr>
		<tr><th>Minute Counter</th><td><xsl:value-of select="p30/@j1" /></td><td><xsl:value-of select="p30/@j2" /></td><td><xsl:value-of select="p30/@j3" /></td><td><xsl:value-of select="p30/@j4" /></td><td><xsl:value-of select="p30/@j5" /></td></tr>
		<tr><th>Overvoltage Protection</th><td><xsl:value-of select="p245/@j1" /></td><td><xsl:value-of select="p245/@j2" /></td><td><xsl:value-of select="p245/@j3" /></td><td><xsl:value-of select="p245/@j4" /></td><td><xsl:value-of select="p245/@j5" /></td></tr>
		
		<h2>Encoder/Initialization Settings</h2>
		<tr><th>Hall sensor invert</th><td><xsl:value-of select="p254/@j1" /></td><td><xsl:value-of select="p254/@j2" /></td><td><xsl:value-of select="p254/@j3" /></td><td><xsl:value-of select="p254/@j4" /></td><td><xsl:value-of select="p254/@j5" /></td></tr>
 		<tr><th>Encoder steps</th><td><xsl:value-of select="p250/@j1" /></td><td><xsl:value-of select="p250/@j2" /></td><td><xsl:value-of select="p250/@j3" /></td><td><xsl:value-of select="p250/@j4" /></td><td><xsl:value-of select="p250/@j5" /></td></tr>
 		<tr><th>Actual encoder position</th><td><xsl:value-of select="p209/@j1" /></td><td><xsl:value-of select="p209/@j2" /></td><td><xsl:value-of select="p209/@j3" /></td><td><xsl:value-of select="p209/@j4" /></td><td><xsl:value-of select="p209/@j5" /></td></tr>
 		<tr><th>Encoder direction</th><td><xsl:value-of select="p251/@j1" /></td><td><xsl:value-of select="p251/@j2" /></td><td><xsl:value-of select="p251/@j3" /></td><td><xsl:value-of select="p251/@j4" /></td><td><xsl:value-of select="p251/@j5" /></td></tr>
 		<tr><th>Actual encoder commutation offset</th><td><xsl:value-of select="p165/@j1" /></td><td><xsl:value-of select="p165/@j2" /></td><td><xsl:value-of select="p165/@j3" /></td><td><xsl:value-of select="p165/@j4" /></td><td><xsl:value-of select="p165/@j5" /></td></tr>
 		<tr><th>Start current</th><td><xsl:value-of select="p177/@j1" /></td><td><xsl:value-of select="p177/@j2" /></td><td><xsl:value-of select="p177/@j3" /></td><td><xsl:value-of select="p177/@j4" /></td><td><xsl:value-of select="p177/@j5" /></td></tr>
 		<tr><th>Init sine mode</th><td><xsl:value-of select="p249/@j1" /></td><td><xsl:value-of select="p249/@j2" /></td><td><xsl:value-of select="p249/@j3" /></td><td><xsl:value-of select="p249/@j4" /></td><td><xsl:value-of select="p249/@j5" /></td></tr>
 		<tr><th>Init sine velocity</th><td><xsl:value-of select="p241/@j1" /></td><td><xsl:value-of select="p241/@j2" /></td><td><xsl:value-of select="p241/@j3" /></td><td><xsl:value-of select="p241/@j4" /></td><td><xsl:value-of select="p241/@j5" /></td></tr>
 		<tr><th>Init sine delay</th><td><xsl:value-of select="p244/@j1" /></td><td><xsl:value-of select="p244/@j2" /></td><td><xsl:value-of select="p244/@j3" /></td><td><xsl:value-of select="p244/@j4" /></td><td><xsl:value-of select="p244/@j5" /></td></tr>
 		<tr><th>velocity treshold for hallFX</th><td><xsl:value-of select="p14/@j1" /></td><td><xsl:value-of select="p14/@j2" /></td><td><xsl:value-of select="p14/@j3" /></td><td><xsl:value-of select="p14/@j4" /></td><td><xsl:value-of select="p14/@j5" /></td></tr>
 		<tr><th>Initialize BLDC</th><td><xsl:value-of select="p15/@j1" /></td><td><xsl:value-of select="p15/@j2" /></td><td><xsl:value-of select="p15/@j3" /></td><td><xsl:value-of select="p15/@j4" /></td><td><xsl:value-of select="p15/@j5" /></td></tr>
 		<tr><th>Commutatuin mode</th><td><xsl:value-of select="p159/@j1" /></td><td><xsl:value-of select="p159/@j2" /></td><td><xsl:value-of select="p159/@j3" /></td><td><xsl:value-of select="p159/@j4" /></td><td><xsl:value-of select="p159/@j5" /></td></tr>
 		<tr><th>Re-Initialization of Sine</th><td><xsl:value-of select="p160/@j1" /></td><td><xsl:value-of select="p160/@j2" /></td><td><xsl:value-of select="p160/@j3" /></td><td><xsl:value-of select="p160/@j4" /></td><td><xsl:value-of select="p160/@j5" /></td></tr>
 		<tr><th>Sine Compensation Factor</th><td><xsl:value-of select="p247/@j1" /></td><td><xsl:value-of select="p247/@j2" /></td><td><xsl:value-of select="p247/@j3" /></td><td><xsl:value-of select="p247/@j4" /></td><td><xsl:value-of select="p247/@j5" /></td></tr>
 		<tr><th>Block PWM scheme</th><td><xsl:value-of select="p167/@j1" /></td><td><xsl:value-of select="p167/@j2" /></td><td><xsl:value-of select="p167/@j3" /></td><td><xsl:value-of select="p167/@j4" /></td><td><xsl:value-of select="p167/@j5" /></td></tr>
 		<tr><th>Init sine block offset CW</th><td><xsl:value-of select="p242/@j1" /></td><td><xsl:value-of select="p242/@j2" /></td><td><xsl:value-of select="p242/@j3" /></td><td><xsl:value-of select="p242/@j4" /></td><td><xsl:value-of select="p242/@j5" /></td></tr>
 		<tr><th>Init sine block offset CCW</th><td><xsl:value-of select="p243/@j1" /></td><td><xsl:value-of select="p243/@j2" /></td><td><xsl:value-of select="p243/@j3" /></td><td><xsl:value-of select="p243/@j4" /></td><td><xsl:value-of select="p243/@j5" /></td></tr>
 		
	</xsl:template>
	<xsl:template match="grippervalue">
	<tr><td><xsl:value-of select="@value"/></td></tr>
	</xsl:template>
	
</xsl:stylesheet>