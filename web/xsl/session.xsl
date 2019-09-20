<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/session">
		<div id="navi"><a href="../">Sessions</a></div>
		
		<h2>Session <xsl:value-of select="@name" /></h2>
		<p><xsl:value-of select="@desc" /></p>
		<form action="./" method="POST"><input type="hidden" name="action" value="TERMINATE"/><input type="submit" value="Terminate session" /></form>
		<h3>Nets</h3>
		<table><tr><th>Net</th><th>Status</th></tr>
			<xsl:apply-templates />
		</table>
		
		<h3>New net</h3>
		<form action="./" method="post"><input type="hidden" name="action" value="LOADNET" />
		<p><textarea name="rpinet" cols="60" rows="10" /></p>
		<p><input type="submit" value="Create" /></p>
		</form>
	</xsl:template>
	<xsl:template match="n">
		<tr><td><a><xsl:attribute name="href">../../nets/<xsl:value-of select="@id">/</xsl:value-of></xsl:attribute><xsl:value-of select="@id"></xsl:value-of></a></td><td><xsl:value-of select="@s"></xsl:value-of></td></tr>
	</xsl:template>

</xsl:stylesheet>