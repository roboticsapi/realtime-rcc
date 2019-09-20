<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/type">
		<div id="navi"><a href="../">Available Types</a></div>
		
		<h2>Type <xsl:value-of select="@name" /></h2>

		<xsl:if test="count(array) = 0 and count(member) = 0">
		Basic type
		</xsl:if>
		
		<xsl:if test="count(array) > 0">
		<xsl:apply-templates select="array" />
		</xsl:if>
		
		<xsl:if test="count(member) > 0">
		<table><tr><th>Name</th><th>Type</th><th>Description</th></tr>
		<xsl:apply-templates select="member" />
		</table>
		</xsl:if>
		
		<h2>Source Code</h2>
		<p><a href="source.java">Java source code</a></p>
	
	</xsl:template>
	
	<xsl:template match="member">
		<tr><td><xsl:value-of select="@name" /></td><td><a><xsl:attribute name="href">/types/<xsl:value-of select="@type" />/</xsl:attribute><xsl:value-of select="@type" /></a></td><td><xsl:value-of select="@description" /></td></tr>
	</xsl:template>
	
	<xsl:template match="array">
		<p>Array of <a><xsl:attribute name="href">/types/<xsl:value-of select="@type" />/</xsl:attribute><xsl:value-of select="@type"></xsl:value-of></a>.</p>
	</xsl:template>
</xsl:stylesheet>