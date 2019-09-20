<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../../layout.xsl"/>

	<xsl:template match="/controller">
		<div id="navi"><a href="../">Devices</a></div>
		
		<h2>youBot Arm <xsl:value-of select="@name" /></h2>
		<dl>
		<dt>Power: </dt><dd><xsl:value-of select="@power" /></dd>
		</dl>
		
		<h2>Robot position</h2>
		<table>
		<tr><td></td><th>Measured</th><th>Commanded</th><td></td><th>MsrKin</th><th>CmdKin</th></tr>
		<tr><th>Joint 1</th><td><xsl:value-of select="joints/@j1" /></td><td><xsl:value-of select="cmdjoints/@j1" /></td><th>X</th><td><xsl:value-of select="frame/@x" /></td><td><xsl:value-of select="cmdframe/@x" /></td></tr>
		<tr><th>Joint 2</th><td><xsl:value-of select="joints/@j2" /></td><td><xsl:value-of select="cmdjoints/@j2" /></td><th>Y</th><td><xsl:value-of select="frame/@y" /></td><td><xsl:value-of select="cmdframe/@y" /></td></tr>
		<tr><th>Joint 3</th><td><xsl:value-of select="joints/@j3" /></td><td><xsl:value-of select="cmdjoints/@j3" /></td><th>Z</th><td><xsl:value-of select="frame/@z" /></td><td><xsl:value-of select="cmdframe/@z" /></td></tr>
		<tr><th>Joint 4</th><td><xsl:value-of select="joints/@j4" /></td><td><xsl:value-of select="cmdjoints/@j4" /></td><th>A</th><td><xsl:value-of select="frame/@a" /></td><td><xsl:value-of select="cmdframe/@a" /></td></tr>
		<tr><th>Joint 5</th><td><xsl:value-of select="joints/@j5" /></td><td><xsl:value-of select="cmdjoints/@j5" /></td><th>B</th><td><xsl:value-of select="frame/@b" /></td><td><xsl:value-of select="cmdframe/@b" /></td></tr>
		<tr><th></th><td></td><td></td><th>C</th><td><xsl:value-of select="frame/@c" /></td><td><xsl:value-of select="cmdframe/@c" /></td></tr>
		</table>

		<a href="kinematics">Kinematic calculations</a>
	</xsl:template>
	<xsl:template match="grippervalue">
	<tr><td><xsl:value-of select="@value"/></td></tr>
	</xsl:template>
	
</xsl:stylesheet>