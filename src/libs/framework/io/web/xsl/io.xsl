<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../layout.xsl"/>

	<xsl:template match="/iointerface">
		<div id="navi"><a href="../">Back</a></div>
		
		<h2>I/O <xsl:value-of select="@name" /></h2>
		<dl>
		<dt>Digital Inputs: </dt><dd><xsl:value-of select="@numdigin" /></dd>
		<dt>Digital Outputs: </dt><dd><xsl:value-of select="@numdigout" /></dd>
		<dt>Analog Inputs: </dt><dd><xsl:value-of select="@numanin" /></dd>
		<dt>Analog Outputs: </dt><dd><xsl:value-of select="@numanout" /></dd>
		</dl>
	
		<h2>Digital Inputs</h2>
		<table>
			<tr><th>I/O</th><th>Value</th></tr>
			<xsl:for-each select="iodigin/*">
				<tr>
					<td><xsl:value-of select="@id"/></td>
					<xsl:call-template name="digio"/>
				</tr>
			</xsl:for-each>
		</table>
		
		<h2>Digital Outputs</h2>
		<table>
			<tr><th>I/O</th><th>Value</th><th>Set</th></tr>
			<xsl:for-each select="iodigout/*">
				<tr>
					<td><xsl:value-of select="@id"/></td>
					<xsl:call-template name="digio"/>
					<td><form method="post" action="io">
						<input type="submit" value="Toggle"/>
						<input type="hidden" name="type" value="digout"/>
						<input type="hidden" name="id"><xsl:attribute name="value"><xsl:value-of select="@id"/></xsl:attribute></input>
						<input type="hidden" name="value"><xsl:attribute name="value"><xsl:value-of select="@value"/></xsl:attribute></input>
					</form></td>
					
				</tr>
			</xsl:for-each>

		</table>
		
		<h2>Analog Inputs</h2>
		<table>
			<tr><th>I/O</th><th>Value</th></tr>
			<xsl:for-each select="ioanin/*">
				<tr><td><xsl:value-of select="@id"/></td><td><xsl:value-of select="@value"/></td></tr>
			</xsl:for-each>
		</table>
		
		<h2>Analog Outputs</h2>
		<table>
			<tr><th>I/O</th><th>Value</th><th>Set</th></tr>
			<xsl:for-each select="ioanout/*">
				<tr>
					<td><xsl:value-of select="@id"/></td>
					<td><xsl:value-of select="@value"/></td>
					<td><form method="post" action="io">
						<input type="hidden" name="type" value="anout"/>
						<input type="hidden" name="id"><xsl:attribute name="value"><xsl:value-of select="@id"/></xsl:attribute></input>
						<input name="value"><xsl:attribute name="value"><xsl:value-of select="@value"/></xsl:attribute></input>
						<input type="submit" value="Set"/>
					</form></td>
					
				</tr>
			</xsl:for-each>

		</table>
	
	</xsl:template>

	<xsl:template name="digio">
	<td>
	<xsl:attribute name="style">background: <xsl:choose><xsl:when
		test="@value = 1">lightgreen</xsl:when><xsl:otherwise>lightpink</xsl:otherwise></xsl:choose>;</xsl:attribute>

	<xsl:choose>
		<xsl:when test="@value = 1">
			on
		</xsl:when>
		<xsl:otherwise>
			off
		</xsl:otherwise>
	</xsl:choose>
	</td>
</xsl:template>
		
</xsl:stylesheet>