<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../layout.xsl"/>

	<xsl:template match="/controller">
		<div id="navi"><a href="../">Devices</a></div>
		
		<h2>SchunkWsg <xsl:value-of select="@name" /></h2>
		<!--
		<img style="float:left; margin-right:40px;" src="../../xsl/schunkwsg/schunkwsg.png" alt="Schunk Wsg Parallel Gripper" />
		//-->
		
		<div>
			<p> </p>
			
			<h3>Current variables</h3>
			
			<p> </p>
			
			<table>
				<tr>
					<td>Busy: </td><td><xsl:value-of select="@busy" /></td>
				</tr><tr>
					<td>Status code: </td><td><xsl:value-of select="@statusCode" /></td>
				</tr><tr>
					<td>Opening width: </td><td><xsl:value-of select="@openingWidth" /></td>
				</tr><tr>
					<td>Velocity: </td><td><xsl:value-of select="@velocity" /></td>
				</tr><tr>
					<td>Force: </td><td><xsl:value-of select="@force" /></td>
				</tr><tr>
					<td>Grasping state: </td><td><xsl:value-of select="@graspingState" /></td>
				</tr>
			</table>
			
			<p> </p>
			
			<form action="./" method="POST">
				<input type="submit" value="Reload"/>
			</form>
			
			<p> </p>
			
			<h3>Manual control</h3>
			
			<p> </p>
			
			<form action="./" method="POST">
				<input type="hidden" name="name"><xsl:attribute name="value"><xsl:value-of select="@name"></xsl:value-of></xsl:attribute></input>
				<input type="hidden" name="partLost" value="true"/>
				<input type="submit" value="Set No Part / Part Lost"/>
			</form>
		</div>
		
		
	</xsl:template>
	
</xsl:stylesheet>