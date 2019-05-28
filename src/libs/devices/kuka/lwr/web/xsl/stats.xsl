<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../../layout.xsl"/>

	<xsl:template match="/controller">
		<div id="navi"><a href="../">FRI Controller <xsl:value-of select="@name" /></a></div>
		
		<h2>Statistics about FRI Controller <xsl:value-of select="@name" /></h2>
		<dl>
		<dt>Status: </dt><dd><xsl:value-of select="@mode" /></dd>
		<dt>Quality: </dt><dd><xsl:value-of select="@quality" /></dd>
		<dt>Control mode: </dt><dd><xsl:value-of select="@controller" /></dd>
		<dt>Cycle time: </dt><dd><xsl:value-of select="@min" /> to <xsl:value-of select="@max" /> ms</dd>
		<dt>Oversampling: </dt><dd><xsl:value-of select="@minignore" /> to <xsl:value-of select="@maxignore" /> values</dd>
		</dl>
		
		<h2>Log</h2>
		<table>
			<tr>
				<th>FRI</th>
				<th>RPI</th>
				<th>Real</th>
				<th>Nsec</th>
				<th>J2</th>
				<th>C2</th>
				<th>Type</th>
				<th>Value</th>
			</tr>
			<xsl:apply-templates />
		</table>
	</xsl:template>
	
	<xsl:template match="//log">
		<tr>
			<td><xsl:value-of select="@fri" /></td>
			<td><xsl:value-of select="@rpi" /></td>
			<td><xsl:value-of select="@real" /></td>
			<td><xsl:value-of select="@nsec" /></td>
			<td><xsl:value-of select="@j2" /></td>
			<td><xsl:value-of select="@c2" /></td>
			<td><xsl:value-of select="@type" /></td>
			<td><xsl:value-of select="@value" /></td>
		</tr>	
	</xsl:template>
	
</xsl:stylesheet>