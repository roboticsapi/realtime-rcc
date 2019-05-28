<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:template match="/">
	<html>
		<body>
digraph net { <br />
<xsl:apply-templates />
}
		</body>
	</html>
	</xsl:template>
	
<xsl:template match="port">
<!--<xsl:value-of select="../@id" /> -&gt; <xsl:value-of select="../@id" />_<xsl:value-of select="@name" /><br />
<xsl:value-of select="@frommodule" />_<xsl:value-of select="@fromport" /> -&gt; <xsl:value-of select="../@id" />_<xsl:value-of select="@name" /><br />-->
<xsl:value-of select="@frommodule" /> -&gt; <xsl:value-of select="../@id" />; <br />
</xsl:template>
</xsl:stylesheet>
