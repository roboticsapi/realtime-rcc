<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/sessions">
		<div id="navi"><a href="../">Start Page</a></div>
		
		<h2>Active Sessions</h2>
		<table><tr><th>Session</th><th>Description</th></tr>
			<xsl:apply-templates />
		</table>
		
		<h2>Create Session</h2>
		<form method="post" action="./">
		<p>Name: <input type="text" name="desc" /> <input type="submit" value="Create" /></p>
		</form>
	</xsl:template>
	
	<xsl:template match="session">
		<tr><td><a><xsl:attribute name="href"><xsl:value-of select="@uri"></xsl:value-of></xsl:attribute><xsl:value-of select="@uri"></xsl:value-of></a></td><td><xsl:value-of select="@desc"></xsl:value-of></td></tr>
	</xsl:template>
</xsl:stylesheet>