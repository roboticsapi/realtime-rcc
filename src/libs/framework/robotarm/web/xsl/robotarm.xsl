<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../layout.xsl"/>

	<xsl:template match="/robotarm">
		<div id="navi"><a href="./">Device</a></div>
		
		<h2>Robot position</h2>
		<table>
		<tr><td></td><th>Measured</th><th>Commanded</th></tr>
		<xsl:apply-templates select="joint"/>
		</table>
	</xsl:template>
	
	<xsl:template match="joint">
		<tr>
			<th>Joint <xsl:value-of select="@nr"/></th>
			<td><xsl:value-of select="@measured" /></td>
			<td><xsl:value-of select="@commanded" /></td>
		</tr>
	</xsl:template>
</xsl:stylesheet>