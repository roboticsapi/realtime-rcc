<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:template match="/">
	<html xmlns:svg="http://www.w3.org/2000/svg">
		<head>
			<link rel="stylesheet" href="/styles.css" />
			<meta http-equiv="Content-Type" content="text/html; charset=UTF-8" />
			<title>Realtime RCC</title>
		</head>
		<body>
			<h1>Realtime RCC</h1>
			<xsl:apply-templates />
		</body>
	</html>
	</xsl:template>
</xsl:stylesheet>
