<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/net">
		<div id="navi"><a href="../">Active Nets</a></div>
		
		<h2>Net <xsl:value-of select="@name" /></h2>
		<h3>Status</h3>
		<xsl:value-of select="@status" />
		
		<h3>Error</h3>
		<table>
			<tr><td>Major:</td><td><xsl:value-of select="errorState/@major"/></td></tr>
			<tr><td>Minor:</td><td><xsl:value-of select="errorState/@minor"/></td></tr>
			<tr><td>Message:</td><td><xsl:value-of select="errorState/text()"/></td></tr>
			<tr><td>Failing block:</td><td><xsl:value-of select="errorState/@block"/></td></tr>
		</table>
		
		<h3>Data</h3>
		<table><tr><th>Name</th><th>Value</th><th>Set value</th></tr>
			<xsl:apply-templates/>
		</table>
		
		<h3>Control</h3>
		<xsl:choose>
			<xsl:when test="@status='REJECTED'">
				<form action="./" method="post"><input type="hidden" name="action" value="UNLOAD" /><input type="submit" value="Unload" /></form>	
			</xsl:when>
			<xsl:when test="@status='LOADING'">
				<form action="./" method="post"><input type="hidden" name="action" value="UNLOAD" /><input type="submit" value="Unload" /></form>
			</xsl:when>
			<xsl:when test="@status='READY'">
				<form action="./" method="post"><input type="hidden" name="action" value="START" /><input type="submit" value="Start" /></form>
				<form action="./" method="post"><input type="hidden" name="action" value="UNLOAD" /><input type="submit" value="Unload" /></form>
				<form action="./" method="post"><input type="hidden" name="action" value="SCHEDULE" />
				<select name="takeovernet" id="takeovernet"></select><input type="submit" value="Schedule" /></form>
				<script type="text/javascript" language="JavaScript">
				/* <![CDATA[ */
			    xmlHttp = new XMLHttpRequest();
			    xmlHttp.open('GET', '/nets/', false);
			    xmlHttp.send(null);
				var nets = xmlHttp.responseXML.getElementsByTagName("net");
				for(var i=0; i<nets.length; i++) {
					var net = nets[i];
			 		if(net.getAttribute("status")=="RUNNING" || net.getAttribute("status")=="READY" || net.getAttribute("status")=="SCHEDULED") {
			 			var uri = nets[i].getAttribute("uri");
			 			var option = document.createElement("option"); 
			 			uri = uri.substr(6, uri.length-7);
			 			option.setAttribute("value", uri);
			 			option.appendChild(document.createTextNode("after " + uri));
			 			document.getElementById("takeovernet").appendChild(option);
			 		}
				}
				/* ]]> */
				</script>
			</xsl:when>
			<xsl:when test="@status='RUNNING'">
				<form action="./" method="post"><input type="hidden" name="action" value="CANCEL" /><input type="submit" value="Cancel" /></form>
				<form action="./" method="post"><input type="hidden" name="action" value="ABORT" /><input type="submit" value="Abort" /></form>
			</xsl:when>
			<xsl:when test="@status='TERMINATED'">
				<form action="./" method="post"><input type="hidden" name="action" value="UNLOAD" /><input type="submit" value="Unload" /></form>
			</xsl:when>
			<xsl:when test="@status='SCHEDULED'">
				<form action="./" method="post"><input type="hidden" name="action" value="UNLOAD" /><input type="submit" value="Unload" /></form>
			</xsl:when>
			<xsl:when test="@status='CANCELLING'">
				<form action="./" method="post"><input type="hidden" name="action" value="ABORT" /><input type="submit" value="Abort" /></form>
			</xsl:when>
			<xsl:otherwise>
				<form action="./" method="post"><input type="hidden" name="action" value="START" /><input type="submit" value="Start" /></form>
				<form action="./" method="post"><input type="hidden" name="action" value="CANCEL" /><input type="submit" value="Cancel" /></form>
				<form action="./" method="post"><input type="hidden" name="action" value="ABORT" /><input type="submit" value="Abort" /></form>
				<form action="./" method="post"><input type="hidden" name="action" value="UNLOAD" /><input type="submit" value="Unload" /></form>			
			</xsl:otherwise>
		</xsl:choose>
	</xsl:template>	
	<xsl:template match="/net/data[starts-with(@key, 'in')]">
		<tr>
			<td><xsl:value-of select="@key"></xsl:value-of></td>
			<td><xsl:value-of select="text()"></xsl:value-of></td>
			<td><form action="./" method="post"><input><xsl:attribute name="name"><xsl:value-of select="@key"/></xsl:attribute></input><input type="submit" value="Set value"/></form></td>
		</tr>		
	</xsl:template>
	<xsl:template match="/net/data[starts-with(@key, 'out')]">
		<tr>
			<td><xsl:value-of select="@key"></xsl:value-of></td>
			<td><xsl:value-of select="text()"></xsl:value-of></td>
			<td></td>
		</tr>		
	</xsl:template>
</xsl:stylesheet>