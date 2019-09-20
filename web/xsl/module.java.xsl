<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:template name="JavaType"><xsl:param name="type" />rpi.types.<xsl:value-of select="substring-before($type,'::')" />.<xsl:choose><xsl:when test="substring($type, string-length($type)-1)='[]'">RPI<xsl:value-of select="substring-after(substring($type, 0, string-length($type)-1), '::')" />Array</xsl:when><xsl:otherwise>RPI<xsl:value-of select="substring-after($type,'::')" /></xsl:otherwise></xsl:choose></xsl:template>
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
<xsl:when test="contains($text,'::')"><xsl:value-of select="substring-before($text, '::')" /></xsl:when>
<xsl:otherwise>core</xsl:otherwise>
</xsl:choose>
</xsl:template>

<xsl:template match="/">
	<xsl:apply-templates select="module" />
</xsl:template>

<xsl:template match="module">
<html><body>
<pre>
package rpi.primitives.<xsl:call-template name="ExtractNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template>;

import de.isse.robotics.rpi.common.Primitive;<xsl:if test="count(inport)>0">
import de.isse.robotics.rpi.common.InPort;</xsl:if><xsl:if test="count(outport)>0">
import de.isse.robotics.rpi.common.OutPort;</xsl:if><xsl:if test="count(parameter)>0">
import de.isse.robotics.rpi.common.Parameter;</xsl:if>

/**
 * <xsl:value-of select="@description" />
 */<xsl:if test="contains(@description, 'deprecated')">
@Deprecated</xsl:if>
public class <xsl:call-template name="StripNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template> extends Primitive {
	/** Type name of the primitive */
	public static final String PRIMITIVE_TYPE = "<xsl:value-of select="@name" />";
	<xsl:apply-templates select="inport" mode="decl" />
	<xsl:apply-templates select="outport" mode="decl" />
	<xsl:apply-templates select="parameter" mode="decl" />
	public <xsl:call-template name="StripNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template>() {
		super(PRIMITIVE_TYPE);
		
		// Add all ports<xsl:apply-templates select="inport" mode="add" /><xsl:apply-templates select="outport" mode="add" />
		
		// Add all parameters<xsl:apply-templates select="parameter" mode="add" />
	}
	
	<xsl:variable name="params"><xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'"><xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template><xsl:text> param</xsl:text><xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template><xsl:text>, </xsl:text></xsl:if></xsl:for-each></xsl:variable>
	<xsl:variable name="nparams"><xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'"><xsl:call-template name="NativeType"><xsl:with-param name="type" select="@type"/></xsl:call-template><xsl:text> param</xsl:text><xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template><xsl:text>, </xsl:text></xsl:if></xsl:for-each></xsl:variable>
	<xsl:variable name="nargs"><xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'"><xsl:call-template name="NativeToJavaType"><xsl:with-param name="type" select="@type" /><xsl:with-param name="value">param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template></xsl:with-param></xsl:call-template><xsl:text>, </xsl:text></xsl:if></xsl:for-each></xsl:variable>
	<xsl:variable name="params2"><xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'"><xsl:variable name="paramname" select="concat('in',@name)"/><xsl:if test="count(../inport[@name=$paramname])=0"><xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template><xsl:text> param</xsl:text><xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template><xsl:text>, </xsl:text></xsl:if></xsl:if></xsl:for-each></xsl:variable>
	<xsl:variable name="nparams2"><xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'"><xsl:variable name="paramname" select="concat('in',@name)"/><xsl:if test="count(../inport[@name=$paramname])=0"><xsl:call-template name="NativeType"><xsl:with-param name="type" select="@type"/></xsl:call-template><xsl:text> param</xsl:text><xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template><xsl:text>, </xsl:text></xsl:if></xsl:if></xsl:for-each></xsl:variable>
	<xsl:variable name="nargs2"><xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'"><xsl:variable name="paramname" select="concat('in',@name)"/><xsl:if test="count(../inport[@name=$paramname])=0"><xsl:call-template name="NativeToJavaType"><xsl:with-param name="type" select="@type" /><xsl:with-param name="value">param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template></xsl:with-param></xsl:call-template><xsl:text>, </xsl:text></xsl:if></xsl:if></xsl:for-each></xsl:variable>
	
	<xsl:if test="$params2 != ''">
	/**
	 * Creates <xsl:call-template name="FirstLower"><xsl:with-param name="text" select="@description" /></xsl:call-template>
	 *<xsl:for-each select="parameter"><xsl:variable name="paramname" select="concat('in',@name)"/><xsl:if test="count(../inport[@name=$paramname])=0">
	 * @param <xsl:call-template name="FirstLower"><xsl:with-param name="text" select="@name" /></xsl:call-template>
	 * <xsl:text>           </xsl:text><xsl:value-of select="@description" />
</xsl:if></xsl:for-each>
	 */ 
	public <xsl:call-template name="StripNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template>(<xsl:value-of select="substring($params2, 0, string-length($params2)-1)" />) {
		this();
		
		// Set the parameters<xsl:for-each select="parameter"><xsl:variable name="paramname" select="concat('in',@name)"/><xsl:if test="count(../inport[@name=$paramname])=0">
		set<xsl:value-of select="@name" />(param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>);</xsl:if></xsl:for-each>		
	}
	</xsl:if>

	<xsl:if test="$nparams2 != $params2">
	/**
	 * Creates <xsl:call-template name="FirstLower"><xsl:with-param name="text" select="@description" /></xsl:call-template>
	 *<xsl:for-each select="parameter"><xsl:variable name="paramname" select="concat('in',@name)"/><xsl:if test="count(../inport[@name=$paramname])=0">
	 * @param <xsl:call-template name="FirstLower"><xsl:with-param name="text" select="@name" /></xsl:call-template>
	 * <xsl:text>           </xsl:text><xsl:value-of select="@description" />
</xsl:if></xsl:for-each>
	 */ 
	public <xsl:call-template name="StripNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template>(<xsl:value-of select="substring($nparams2, 0, string-length($nparams2)-1)" />) {
		this(<xsl:value-of select="substring($nargs2, 0, string-length($nargs2)-1)" />);
	}
	</xsl:if>
	
	<xsl:if test="$params != $params2">
	/**
	 * Creates <xsl:call-template name="FirstLower"><xsl:with-param name="text" select="@description" /></xsl:call-template>
	 *<xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'">
	 * @param param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>
	 * <xsl:text>           </xsl:text><xsl:value-of select="@description" />
</xsl:if></xsl:for-each>
	 */ 
	public <xsl:call-template name="StripNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template>(<xsl:value-of select="substring($params, 0, string-length($params)-1)" />) {
		this();
		
		// Set the parameters<xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'">
		set<xsl:value-of select="@name" />(param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>);</xsl:if></xsl:for-each>		
	}
	</xsl:if>

	<xsl:if test="$nparams != $params and $nparams != $nparams2">
	/**
	 * Creates <xsl:call-template name="FirstLower"><xsl:with-param name="text" select="@description" /></xsl:call-template>
	 *<xsl:for-each select="parameter"><xsl:if test="@type != 'unknown'">
	 * @param param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>
	 * <xsl:text>           </xsl:text><xsl:value-of select="@description" />
</xsl:if></xsl:for-each>
	 */ 
	public <xsl:call-template name="StripNamespace"><xsl:with-param name="text" select="@name" /></xsl:call-template>(<xsl:value-of select="substring($nparams, 0, string-length($nparams)-1)" />) {
		this(<xsl:value-of select="substring($nargs, 0, string-length($nargs)-1)" />);
	}
	</xsl:if>
	
	<xsl:apply-templates select="inport" mode="member" />
	<xsl:apply-templates select="outport" mode="member" />
	<xsl:apply-templates select="parameter" mode="member" />
}
</pre>
</body></html>
</xsl:template>
	
<xsl:template match="inport" mode="decl">
	/** <xsl:value-of select="@description" /> */
	private final InPort <xsl:value-of select="@name" /> = new InPort("<xsl:value-of select="@name" />");
</xsl:template>
<xsl:template match="outport" mode="decl">
	/** <xsl:value-of select="@description" /> */
	private final OutPort <xsl:value-of select="@name" /> = new OutPort("<xsl:value-of select="@name" />");
</xsl:template>
<xsl:template match="parameter" mode="decl"><xsl:if test="@type != 'unknown'">
	/** <xsl:value-of select="@description" /> */
	private final Parameter&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt; param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template> = new Parameter&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt;("<xsl:value-of select="@name" />", <xsl:call-template name="JavaValue"><xsl:with-param name="type" select="@type" /><xsl:with-param name="value" select="@value" /></xsl:call-template>);
</xsl:if></xsl:template>

<xsl:template match="inport" mode="add">
		add(<xsl:value-of select="@name" />);</xsl:template>
<xsl:template match="outport" mode="add">
		add(<xsl:value-of select="@name" />);</xsl:template>
<xsl:template match="parameter" mode="add"><xsl:if test="@type != 'unknown'">
		add(param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>);</xsl:if></xsl:template>

<xsl:template match="inport" mode="member">
	/**
	 * <xsl:value-of select="@description" />
	 *
	 * @return the input port of the block
	 */
	public final InPort get<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>() {
		return this.<xsl:value-of select="@name" />; 
	}
</xsl:template>
<xsl:template match="outport" mode="member">
	/**
	 * <xsl:value-of select="@description" />
	 * 
	 * @return the output port of the block
	 */
	public final OutPort get<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>() {
		return this.<xsl:value-of select="@name" />; 
	}
</xsl:template>
<xsl:template match="parameter" mode="member"><xsl:if test="@type != 'unknown'">
	/**
	 * <xsl:value-of select="@description" />
	 * 
	 * @return the parameter of the block
	 */
	public final Parameter&lt;<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template>&gt; get<xsl:value-of select="@name" />() {
		return this.param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>; 
	}
	
	/**
	 * Sets a parameter of the block:
	 * <xsl:value-of select="@description" />
	 * 
	 * @param value new value of the parameter
	 */	
	public final void set<xsl:value-of select="@name" />(<xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template> value) { 
		this.param<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>.setValue(value); 
	}
	
	/**
	 * Sets a parameter of the block:
	 * <xsl:value-of select="@description" />
	 * 
	 * @param value new value of the parameter
	 */	
	public final void set<xsl:value-of select="@name" />(<xsl:call-template name="NativeType"><xsl:with-param name="type" select="@type"/></xsl:call-template> value) { 
		this.set<xsl:value-of select="@name" />(<xsl:call-template name="NativeToJavaType"><xsl:with-param name="type" select="@type" /><xsl:with-param name="value">value</xsl:with-param></xsl:call-template>); 
	}
</xsl:if></xsl:template>
<xsl:template match="parameter" mode="consparam"><xsl:if test="@type != 'unknown'"><xsl:call-template name="JavaType"><xsl:with-param name="type" select="@type"/></xsl:call-template><xsl:value-of select="' param'"/><xsl:call-template name="FirstLower"><xsl:with-param name="text" select="@name" /></xsl:call-template>, </xsl:if></xsl:template>
<xsl:template match="parameter" mode="consset"><xsl:if test="@type != 'unknown'">
		set<xsl:value-of select="@name" />(<xsl:call-template name="FirstLower"><xsl:with-param name="text" select="@name" /></xsl:call-template>);</xsl:if></xsl:template>
</xsl:stylesheet>