<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/extensions">
		<div id="navi"><a href="../">Start Page</a></div>
		
		<h2>Loaded Extensions</h2>
		<table><tr><th>Name</th><th>File name</th></tr>
			<xsl:apply-templates />
		</table>
		
		<h2>Load Extension</h2>
		<form action="./" method="POST">
		Name: <input type="text" name="extension" /><input type="submit" value="Load" />
		</form>
	</xsl:template>
	
	<xsl:template match="extension">
		<tr><td><a><xsl:attribute name="href"><xsl:value-of select="@name" />/</xsl:attribute><xsl:value-of select="@name" /></a></td><td><xsl:value-of select="@file" /></td></tr>
	</xsl:template>
</xsl:stylesheet>