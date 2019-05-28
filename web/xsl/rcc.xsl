<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/device">
		<div id="navi"><a href="../">Devices</a></div>
		
		<h2>RCC</h2>
		<xsl:if test="@estop='true'"><div style="background:#f00;color:#fff;text-align:center;font-size:2em;">EMERGENCY STOP</div></xsl:if>
		<p><a href="parameters">Parameters</a></p>
	</xsl:template>
	
</xsl:stylesheet>