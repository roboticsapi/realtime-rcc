<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/modules">
		<div id="navi"><a href="../">Start Page</a></div>
		
		<h2>Available Modules</h2>
		<ul>
			<xsl:apply-templates />
		</ul>
	</xsl:template>
	
	<xsl:template match="module">
		<li><a><xsl:attribute name="href"><xsl:value-of select="@uri"></xsl:value-of></xsl:attribute><xsl:value-of select="@name"></xsl:value-of></a></li>
	</xsl:template>
</xsl:stylesheet>