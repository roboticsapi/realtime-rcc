<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../../layout.xsl"/>

	<xsl:template match="/controller">
		<div id="navi"><a href="../">Devices</a></div>
		
		<h2>FRI Controller <xsl:value-of select="@name" /></h2>
		<dl>
		<dt>Status: </dt><dd><xsl:value-of select="@mode" /></dd>
		<dt>Quality: </dt><dd><xsl:value-of select="@quality" /></dd>
		<dt>Control mode: </dt><dd><xsl:value-of select="@controller" /></dd>
		</dl>
		<p><a href="stats">Communication Statistics</a></p>
		<p><a href="parameters">Parameters</a></p>
						
		<h2>Robot position</h2>
		<table>
		<tr><td></td><th>Measured</th><th>Commanded</th><td></td><th>MsrKin</th><th>CmdKin</th><th>MsrFri</th></tr>
		<tr><th>Joint 1</th><td><xsl:value-of select="joints/@j1" /></td><td><xsl:value-of select="cmdjoints/@j1" /></td><th>X</th><td><xsl:value-of select="frame/@x" /></td><td><xsl:value-of select="cmdframe/@x" /></td><td><xsl:value-of select="friframe/@x" /></td></tr>
		<tr><th>Joint 2</th><td><xsl:value-of select="joints/@j2" /></td><td><xsl:value-of select="cmdjoints/@j2" /></td><th>Y</th><td><xsl:value-of select="frame/@y" /></td><td><xsl:value-of select="cmdframe/@y" /></td><td><xsl:value-of select="friframe/@y" /></td></tr>
		<tr><th>Joint 3</th><td><xsl:value-of select="joints/@j3" /></td><td><xsl:value-of select="cmdjoints/@j3" /></td><th>Z</th><td><xsl:value-of select="frame/@z" /></td><td><xsl:value-of select="cmdframe/@z" /></td><td><xsl:value-of select="friframe/@z" /></td></tr>
		<tr><th>Joint 4</th><td><xsl:value-of select="joints/@j4" /></td><td><xsl:value-of select="cmdjoints/@j4" /></td><th>A</th><td><xsl:value-of select="frame/@a" /></td><td><xsl:value-of select="cmdframe/@a" /></td><td><xsl:value-of select="friframe/@a" /></td></tr>
		<tr><th>Joint 5</th><td><xsl:value-of select="joints/@j5" /></td><td><xsl:value-of select="cmdjoints/@j5" /></td><th>B</th><td><xsl:value-of select="frame/@b" /></td><td><xsl:value-of select="cmdframe/@b" /></td><td><xsl:value-of select="friframe/@b" /></td></tr>
		<tr><th>Joint 6</th><td><xsl:value-of select="joints/@j6" /></td><td><xsl:value-of select="cmdjoints/@j6" /></td><th>C</th><td><xsl:value-of select="frame/@c" /></td><td><xsl:value-of select="cmdframe/@c" /></td><td><xsl:value-of select="friframe/@c" /></td></tr>
		<tr><th>Joint 7</th><td><xsl:value-of select="joints/@j7" /></td><td><xsl:value-of select="cmdjoints/@j7" /></td><th>Alpha</th><td><xsl:value-of select="frame/@alpha" /></td><td><xsl:value-of select="cmdframe/@alpha" /></td><td> </td></tr>
		</table>
		
		<a href="kinematics">Kinematic calculations</a>
		
		<h2>Stiffness configuration</h2>
				<table>
		<tr><td></td><th>Stiffness</th><th>Damping</th><td></td><th>Stiffness</th><th>Damping</th></tr>
		<tr><th>Joint 1</th><td><xsl:value-of select="jntstiff/@j1" /></td><td><xsl:value-of select="jntdamp/@j1" /></td><th>X</th><td><xsl:value-of select="cartstiff/@x" /></td><td><xsl:value-of select="cartdamp/@x" /></td></tr>
		<tr><th>Joint 2</th><td><xsl:value-of select="jntstiff/@j2" /></td><td><xsl:value-of select="jntdamp/@j2" /></td><th>Y</th><td><xsl:value-of select="cartstiff/@y" /></td><td><xsl:value-of select="cartdamp/@y" /></td></tr>
		<tr><th>Joint 3</th><td><xsl:value-of select="jntstiff/@j3" /></td><td><xsl:value-of select="jntdamp/@j3" /></td><th>Z</th><td><xsl:value-of select="cartstiff/@z" /></td><td><xsl:value-of select="cartdamp/@z" /></td></tr>
		<tr><th>Joint 4</th><td><xsl:value-of select="jntstiff/@j4" /></td><td><xsl:value-of select="jntdamp/@j4" /></td><th>A</th><td><xsl:value-of select="cartstiff/@a" /></td><td><xsl:value-of select="cartdamp/@a" /></td></tr>
		<tr><th>Joint 5</th><td><xsl:value-of select="jntstiff/@j5" /></td><td><xsl:value-of select="jntdamp/@j5" /></td><th>B</th><td><xsl:value-of select="cartstiff/@b" /></td><td><xsl:value-of select="cartdamp/@b" /></td></tr>
		<tr><th>Joint 6</th><td><xsl:value-of select="jntstiff/@j6" /></td><td><xsl:value-of select="jntdamp/@j6" /></td><th>C</th><td><xsl:value-of select="cartstiff/@c" /></td><td><xsl:value-of select="cartdamp/@c" /></td></tr>
		<tr><th>Joint 7</th><td><xsl:value-of select="jntstiff/@j7" /></td><td><xsl:value-of select="jntdamp/@j7" /></td></tr>
		</table>
		
	
	</xsl:template>
	
</xsl:stylesheet>