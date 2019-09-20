<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:template name="JavaType"><xsl:param name="type" /><xsl:choose><xsl:when test="substring($type, string-length($type)-1)='[]'">RPI<xsl:value-of select="substring-after(substring($type, 0, string-length($type)-1), '::')" />Array</xsl:when><xsl:otherwise>RPI<xsl:value-of select="substring-after($type,'::')" /></xsl:otherwise></xsl:choose></xsl:template>
<xsl:template name="JavaValue"><xsl:param name="type" /><xsl:param name="value" /><xsl:text>new </xsl:text><xsl:call-template name="JavaType"><xsl:with-param name="type" select="$type" /></xsl:call-template>("<xsl:value-of select="$value" />")</xsl:template>
<xsl:template name="NativeType"><xsl:param name="type" /><xsl:choose><xsl:when test="$type='Core::bool'">Boolean</xsl:when><xsl:when test="$type='Core::int'">Integer</xsl:when><xsl:when test="$type='Core::string'">String</xsl:when><xsl:when test="$type='Core::double'">Double</xsl:when><xsl:otherwise>String</xsl:otherwise></xsl:choose></xsl:template>
<xsl:template name="NativeToJavaType"><xsl:param name="type" /><xsl:param name="value" /><xsl:text>new </xsl:text><xsl:call-template name="JavaType"><xsl:with-param name="type" select="$type" /></xsl:call-template>(<xsl:value-of select="$value" />)</xsl:template>

<xsl:template name="FirstUpper"><xsl:param name="text" /><xsl:value-of select="translate(substring($text,1,1),'abcdefghijklmnopqrstuvwxyz','ABCDEFGHIJKLMNOPQRSTUVWXYZ')" /><xsl:value-of select="substring($text,2)" /></xsl:template>
<xsl:template name="FirstLower"><xsl:param name="text" /><xsl:value-of select="translate(substring($text,1,1),'ABCDEFGHIJKLMNOPQRSTUVWXYZ','abcdefghijklmnopqrstuvwxyz')" /><xsl:value-of select="substring($text,2)" /></xsl:template>

<xsl:template name="StripNamespace"><xsl:param name="text" />
<xsl:choose>
<xsl:when test="contains($text,'::')"><xsl:call-template name="StripNamespace"><xsl:with-param name="text" select="substring-after($text, '::')" /></xsl:call-template></xsl:when>
<xsl:otherwise><xsl:value-of select="$text" /></xsl:otherwise>
</xsl:choose>
</xsl:template>

<xsl:template name="ExtractNamespace"><xsl:param name="text" />
<xsl:choose>
<xsl:when test="contains($text,'::')"><xsl:value-of select="translate(substring-before($text,'::'),'ABCDEFGHIJKLMNOPQRSTUVWXYZ','abcdefghijklmnopqrstuvwxyz')" /></xsl:when>
<xsl:otherwise>core</xsl:otherwise>
</xsl:choose>
</xsl:template>

<xsl:template match="/">
	<xsl:apply-templates select="module" />
</xsl:template>

<xsl:template match="module">
<html><body>
<pre>package org.roboticsapi.facet.javarcc.primitives.<xsl:call-template name="ExtractNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template>;
<xsl:if test="count(sensor) + count(actuator)>0">
import java.util.Set;
</xsl:if>
<xsl:if test="count(inport)>0">
import org.roboticsapi.facet.javarcc.JInPort;</xsl:if><xsl:if test="count(outport)>0">
import org.roboticsapi.facet.javarcc.JOutPort;</xsl:if><xsl:if test="count(parameter)>0">
import org.roboticsapi.facet.javarcc.JParameter;</xsl:if>
import org.roboticsapi.facet.javarcc.JPrimitive;<xsl:if test="count(sensor) + count(actuator)>0">
import org.roboticsapi.facet.javarcc.devices.JDevice;</xsl:if>
import org.roboticsapi.facet.runtime.rpi.core.types.*;
import org.roboticsapi.facet.runtime.rpi.world.types.*;

/**
 * <xsl:value-of select="@description" />
 */<xsl:if test="contains(@description, 'deprecated')">
@Deprecated</xsl:if>
public class J<xsl:call-template name="StripNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template> extends JPrimitive {
	<xsl:apply-templates select="inport" mode="decl" />
	<xsl:apply-templates select="outport" mode="decl" />
	<xsl:apply-templates select="parameter" mode="decl" />

	<xsl:apply-templates select="sensor" mode="decl" />

	@Override
	public void checkParameters() throws IllegalArgumentException {
		// TODO: do parameter checks
	}
	
	@Override
	public void updateData() {
		// TODO: perform computations based on local variables and InPort values, 
		// write local variables and OutPorts
	}
	<xsl:apply-templates select="actuator" mode="decl" />
}
</pre>
</body></html>
</xsl:template>
	
<xsl:template match="inport" mode="decl">
	/** <xsl:value-of select="@description" /> */
	final JInPort&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt; <xsl:value-of select="@name" /> = add("<xsl:value-of select="@name" />", new JInPort&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt;());
</xsl:template>
<xsl:template match="outport" mode="decl">
	/** <xsl:value-of select="@description" /> */
	final JOutPort&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt; <xsl:value-of select="@name" /> = add("<xsl:value-of select="@name" />", new JOutPort&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt;());
</xsl:template>
<xsl:template match="parameter" mode="decl"><xsl:if test="@type != 'unknown'">
	/** <xsl:value-of select="@description" /> */
	final JParameter&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt; prop<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template> = add("<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>", new JParameter&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt;(<xsl:call-template name="JavaValue"><xsl:with-param name="type" select="@type" /><xsl:with-param name="value" select="@value" /></xsl:call-template>));
</xsl:if></xsl:template>
<xsl:template match="sensor" mode="decl">

	@Override
	public Set&lt;JDevice&gt; getSensors() {
		// TODO: return the JDevices this JPrimitive accesses
		return super.getSensors();
	}
	
	@Override
	public void readSensor() {
		// TODO: read sensor data from a JDevice mentioned in getSensors() and 
		// write it to local variables
		super.readSensor();
	}
</xsl:template>
<xsl:template match="actuator" mode="decl">

	@Override
	public Set&lt;JDevice&gt; getActuators() {
		// TODO: return the JDevices this JPrimitive accesses
		return super.getActuators();
	}

	@Override
	public void writeActuator() {
		// TODO: send control data from local variables to a JDevice mentioned 
		// in getActuators()
		super.writeActuator();
	}
</xsl:template>
</xsl:stylesheet>

