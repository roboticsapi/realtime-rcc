<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/devices">
		<div id="navi"><a href="../">Start Page</a></div>
		
		<h2>Loaded Devices</h2>
		<table><tr><th>Name</th><th>Type</th><th>State</th><th>Interfaces</th><th>Parameters</th><th>Remove</th></tr>
			<xsl:apply-templates />
		</table>
	</xsl:template>
	
	<xsl:template match="device">
		<tr>
			<td><a><xsl:attribute name="href"><xsl:value-of select="@name" />/</xsl:attribute><xsl:value-of select="@name" /></a></td>
			<td><xsl:value-of select="@type"/></td>
			<td><xsl:value-of select="@state"/></td>
			<td><xsl:for-each select="interface"><a><xsl:attribute name="href"><xsl:value-of select="../@name"/>/<xsl:value-of select="@name"/></xsl:attribute> <xsl:value-of select="@name"/></a><xsl:text> </xsl:text></xsl:for-each></td>
			<td><a><xsl:attribute name="href"><xsl:value-of select="@name"/>/parameters</xsl:attribute>Parameters</a></td>
			<td><form action="./" method="POST">
				<input type="hidden" name="name"><xsl:attribute name="value"><xsl:value-of select="@name"></xsl:value-of></xsl:attribute></input>
				<input type="hidden" name="action" value="remove"/>
				<input type="submit" value="Remove"/>
			</form></td>
		</tr>
	</xsl:template>
</xsl:stylesheet>