<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../../layout.xsl"/>

	<xsl:template match="/controller">
		<div id="navi"><a href="../">Devices</a></div>
		
		<h2>youBot Base <xsl:value-of select="@name" /></h2>
		
		<dl>
		<dt>X</dt><dd><xsl:value-of select="@x" /></dd>
		<dt>Y</dt><dd><xsl:value-of select="@y" /></dd>
		<dt>Yaw</dt><dd><xsl:value-of select="@yaw" /></dd>
		</dl>
		
		<p><a href="parameters">Parameters</a></p>
		
	</xsl:template>
	
</xsl:stylesheet>