<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:include href="layout.xsl"/>

	<xsl:template match="/nets">
		<div id="navi"><a href="../">Start Page</a></div>
		
		<h2>Active Nets</h2>
		<table><tr><th>Net</th><th>Status</th><th>Description</th></tr>
			<xsl:apply-templates />
		</table>
		<script type="text/javascript" language="JavaScript">
		/* <![CDATA[ */
		function unloadNets() {
			var xmlHttp = new XMLHttpRequest();
			if (xmlHttp) {
			    xmlHttp.open('GET', '/nets/', false);
			    xmlHttp.send(null);	    
				var nets = xmlHttp.responseXML.getElementsByTagName("net");
				for(var i=0; i<nets.length; i++) {
					var net = nets[i];
			 		if(net.getAttribute("status")=="REJECTED" || net.getAttribute("status")=="TERMINATED") {
			 			var uri = nets[i].getAttribute("uri");
			 			var xmlHttp = new XMLHttpRequest();
			 			xmlHttp.open('POST', uri, false);
			 			xmlHttp.send('action=UNLOAD');
			 		}
				}
				setTimeout("window.location.reload();", 1000);
			}
		}
		/* ]]> */
		</script>
		<button onclick="unloadNets()">Unload terminated / rejected nets</button>
		
		<h2>Create Net</h2>
		<form method="post" action="./">
		<p><textarea rows="15" cols="80" name="rpinet"></textarea></p>
		<p>Description: <input type="text" name="desc" size="50" /> <input type="submit" value="Create" /></p>
		</form>
	</xsl:template>
	
	<xsl:template match="net">
		<tr><td><a><xsl:attribute name="href"><xsl:value-of select="@uri"></xsl:value-of></xsl:attribute><xsl:value-of select="@uri"></xsl:value-of></a></td><td><xsl:value-of select="@status"></xsl:value-of></td><td><xsl:value-of select="@desc"></xsl:value-of></td></tr>
	</xsl:template>
</xsl:stylesheet>