<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="../layout.xsl"/>

	<xsl:template match="/kinematics">
		<div id="navi"><a href="./">Device</a></div>
		
		<h2>Kinematics calculations</h2>
		<form action="armkinematics" method="get">
		<table>
		<tr><th colspan="3">Joints</th></tr>
		<xsl:apply-templates select="joint"/>
		<tr><th colspan="3">Frame</th></tr>
		<tr>
			<th>X</th>
			<td><xsl:value-of select="frame/@x" /></td>
			<td><input type="text" name="x"><xsl:attribute name="onfocus">if(this.value=="") this.value="<xsl:value-of select="frame/@x" />";</xsl:attribute></input></td>
		</tr>
		<tr>
			<th>Y</th>
			<td><xsl:value-of select="frame/@y" /></td>
			<td><input type="text" name="y"><xsl:attribute name="onfocus">if(this.value=="") this.value="<xsl:value-of select="frame/@y" />";</xsl:attribute></input></td>
		</tr>
		<tr>
			<th>Z</th>
			<td><xsl:value-of select="frame/@z" /></td>
			<td><input type="text" name="z"><xsl:attribute name="onfocus">if(this.value=="") this.value="<xsl:value-of select="frame/@z" />";</xsl:attribute></input></td>
		</tr>
		<tr>
			<th>A</th>
			<td><xsl:value-of select="frame/@a" /></td>
			<td><input type="text" name="a"><xsl:attribute name="onfocus">if(this.value=="") this.value="<xsl:value-of select="frame/@a" />";</xsl:attribute></input></td>
		</tr>
		<tr>
			<th>B</th>
			<td><xsl:value-of select="frame/@b" /></td>
			<td><input type="text" name="b"><xsl:attribute name="onfocus">if(this.value=="") this.value="<xsl:value-of select="frame/@b" />";</xsl:attribute></input></td>
		</tr>
		<tr>
			<th>C</th>
			<td><xsl:value-of select="frame/@c" /></td>
			<td><input type="text" name="c"><xsl:attribute name="onfocus">if(this.value=="") this.value="<xsl:value-of select="frame/@c" />";</xsl:attribute></input></td>
		</tr>
		</table>
		<input type="submit" value="Calculate" />
		</form>

	</xsl:template>
	
	<xsl:template match="joint">
		<tr>
			<th>Joint <xsl:value-of select="@nr"/></th>
			<td><xsl:value-of select="@value" /></td>
			<td><input type="text"><xsl:attribute name="name">j<xsl:value-of select="@nr"/></xsl:attribute><xsl:attribute name="onfocus">if(this.value=="") this.value="<xsl:value-of select="@value" />";</xsl:attribute></input></td>
			
		</tr>
	</xsl:template>
</xsl:stylesheet>