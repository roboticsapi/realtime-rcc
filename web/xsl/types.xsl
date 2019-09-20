<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/types">
		<div id="navi"><a href="../">Start Page</a></div>
		
		<h2>Available Data Types</h2>
		<table><tr><th>Name</th><th>Kind</th></tr>
			<xsl:apply-templates />
		</table>
	</xsl:template>
	
	<xsl:template match="type">
		<tr><td><a><xsl:attribute name="href">/types/<xsl:value-of select="@name">/</xsl:value-of></xsl:attribute><xsl:value-of select="@name"></xsl:value-of></a></td><td><xsl:value-of select="@kind" /></td></tr>
	</xsl:template>
</xsl:stylesheet>