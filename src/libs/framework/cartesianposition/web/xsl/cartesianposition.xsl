<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../layout.xsl"/>

	<xsl:template match="/cartesianposition">
		<div id="navi"><a href="./">Device</a></div>
		
		<h2>Position</h2>
		<table>
		<tr><td></td><th>Measured</th><th>Commanded</th></tr>
		<tr><td>X</td><td><xsl:value-of select="measured/@x"/></td><td><xsl:value-of select="commanded/@x"/></td></tr>
		<tr><td>Y</td><td><xsl:value-of select="measured/@y"/></td><td><xsl:value-of select="commanded/@y"/></td></tr>
		<tr><td>Z</td><td><xsl:value-of select="measured/@z"/></td><td><xsl:value-of select="commanded/@z"/></td></tr>
		<tr><td>A</td><td><xsl:value-of select="measured/@a"/></td><td><xsl:value-of select="commanded/@a"/></td></tr>
		<tr><td>B</td><td><xsl:value-of select="measured/@b"/></td><td><xsl:value-of select="commanded/@b"/></td></tr>
		<tr><td>C</td><td><xsl:value-of select="measured/@c"/></td><td><xsl:value-of select="commanded/@c"/></td></tr>
		</table>
		
		<h2>Velocity</h2>		
		<table>		
		<tr><td></td><th>Measured</th><th>Commanded</th></tr>
		<tr><td>X</td><td><xsl:value-of select="measured/@vx"/></td><td><xsl:value-of select="commanded/@vx"/></td></tr>	
		<tr><td>Y</td><td><xsl:value-of select="measured/@vy"/></td><td><xsl:value-of select="commanded/@vy"/></td></tr>		<tr><td>Z</td><td><xsl:value-of select="measured/@vz"/></td><td><xsl:value-of select="commanded/@vz"/></td></tr>		<tr><td>RX</td><td><xsl:value-of select="measured/@rx"/></td><td><xsl:value-of select="commanded/@rx"/></td></tr>		<tr><td>RY</td><td><xsl:value-of select="measured/@ry"/></td><td><xsl:value-of select="commanded/@ry"/></td></tr>		<tr><td>RZ</td><td><xsl:value-of select="measured/@rz"/></td><td><xsl:value-of select="commanded/@rz"/></td></tr>		</table>
		
	</xsl:template>
</xsl:stylesheet>