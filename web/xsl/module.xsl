<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/module">
		<div id="navi"><a href="../">Available Modules</a></div>
		
		<h2>Module <xsl:value-of select="@name" /></h2>
		<h3>Description</h3>
		<p><xsl:value-of select="@description" /></p>
		<h3>Parameters</h3>
		<table><tr><th>Name</th><th>Type</th><th>Description</th></tr>
		<xsl:apply-templates select="parameter" />
		</table>
		
		<h3>Input ports</h3>
		<table><tr><th>Name</th><th>Type</th><th>Description</th></tr>
		<xsl:apply-templates select="inport" />
		</table>
		
		<h3>Output ports</h3>
		<table><tr><th>Name</th><th>Type</th><th>Description</th></tr>
		<xsl:apply-templates select="outport" />
		</table>
		
		<h2>Source Code</h2>
		<a href="source.java">Java interface source</a>
	</xsl:template>
	
	<xsl:template match="inport">
		<tr><td><xsl:value-of select="@name" /></td><td><xsl:value-of select="@type" /></td><td><xsl:value-of select="@description" /></td></tr>
	</xsl:template>
	<xsl:template match="outport">
		<tr><td><xsl:value-of select="@name" /></td><td><xsl:value-of select="@type" /></td><td><xsl:value-of select="@description" /></td></tr>
	</xsl:template>
	<xsl:template match="parameter">
		<tr><td><xsl:value-of select="@name" /></td><td><xsl:value-of select="@type" /></td><td><xsl:value-of select="@description" /></td></tr>
	</xsl:template>
</xsl:stylesheet>