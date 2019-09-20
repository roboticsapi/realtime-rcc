<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0"
	xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl" />

	<xsl:template match="/parameters">
		<div id="navi">
			<a href="./">Driver Informations</a>
		</div>

		<h2>Parameters</h2>
		<form method="post">
		<table>
			<tr>
				<th>Name</th>
				<th>Value</th>
			</tr>
			<xsl:apply-templates />
		</table>
		<input type="submit" value="Change"/>
		</form>

	</xsl:template>

	<xsl:template match="parameter">
		<tr>
			<td>
				<xsl:value-of select="@name" />
			</td>
			<td>
				<xsl:choose>
					<xsl:when test="@mutable = 'true'">
						<input type="text">
							<xsl:attribute name="value"><xsl:value-of select="text()"/></xsl:attribute>
							<xsl:attribute name="name"><xsl:value-of select="@name"/></xsl:attribute>
						</input>
					</xsl:when>
					<xsl:otherwise>
						<xsl:value-of select="text()" />
					</xsl:otherwise>
				</xsl:choose>
			</td>
		</tr>
	</xsl:template>
</xsl:stylesheet>