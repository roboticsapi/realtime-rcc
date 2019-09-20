<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../layout.xsl"/>

	<xsl:template match="/robotbase">
		<div id="navi"><a href="./">Device</a></div>
		
		<h2>RobotBase</h2>
		
		<table>
		<tr><td></td><th>Position</th><th>Velocity</th></tr>
		<tr><td>X</td><td><xsl:value-of select="position/@x"/></td><td><xsl:value-of select="velocity/@x"/></td></tr>
		<tr><td>Y</td><td><xsl:value-of select="position/@y"/></td><td><xsl:value-of select="velocity/@y"/></td></tr>
		<tr><td>Yaw</td><td><xsl:value-of select="position/@theta"/></td><td><xsl:value-of select="velocity/@theta"/></td></tr>
		</table>
		
		
		
	</xsl:template>
	
</xsl:stylesheet>