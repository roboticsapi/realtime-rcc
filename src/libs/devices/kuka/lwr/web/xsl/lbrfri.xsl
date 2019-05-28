<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../../layout.xsl"/>

	<xsl:template match="/controllers">
		<div id="navi"><a href="../">Loaded Extensions</a></div>
		
		<h2>FRI Controllers</h2>
		<ul>
			<xsl:apply-templates />
		</ul>
		
		<h2>Add Controller</h2>
		<form action="/devices/" method="POST">
			<input type="hidden" name="type" value="lbr_joint"/>
			Robot name: <input type="text" name="name" />
			IP: <input type="text" name="ip" />
			Port: <input type="text" name="port" size="5" maxlength="5" />
			<input type="submit" value="Add" />
		</form>
	</xsl:template>
	
	<xsl:template match="controller">
		<li><a><xsl:attribute name="href"><xsl:value-of select="@uri" /></xsl:attribute><xsl:value-of select="@name" /></a></li>
	</xsl:template>
</xsl:stylesheet>