<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<xsl:template match="/type">
<html><body>
<pre>
package rpi.types.<xsl:call-template name="JavaPackage"><xsl:with-param name="name" select="@name" /></xsl:call-template>;
<xsl:if test="count(array) = 0 and count(member) = 0">
// Basic type <xsl:value-of select="@name" />
// Nothing to generate here.
</xsl:if><xsl:if test="count(array) > 0">
import rpi.types.ArrayType;

/**
 * Array of <xsl:call-template name="TypeName"><xsl:with-param name="name" select="array/@type" /></xsl:call-template>
 * a.k.a <xsl:value-of select="@name" />
 */
public class <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template> extends ArrayType&lt;<xsl:call-template name="JavaType"><xsl:with-param name="name" select="array/@type" /></xsl:call-template>&gt; {

	/**
	 * Creates an array of the given size and initializes it with the given contents
	 *
	 * @param capacity
	 *            size of the array
	 * @param value
	 *            initial value (in RPI syntax)
	 */
	public <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>(int capacity, String value) {
		this(capacity);
		consumeString(value);
	}
	
	/**
	 * Creates an array and initializes it with the given contents
	 *
	 * @param value
	 *            initial value (in RPI syntax)
	 */
	public <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>(String value) {
		super(value);
	}
	
	/**
	 * Creates an array of the given size
	 *
	 * @param capacity
	 *            size of the array
	 */
	public <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>(int capacity) {
		super(capacity);
	} 

	@Override
	protected <xsl:call-template name="JavaType"><xsl:with-param name="name" select="array/@type" /></xsl:call-template> getInitialValue() {
		return new <xsl:call-template name="JavaType"><xsl:with-param name="name" select="array/@type" /></xsl:call-template>();
	}
}
</xsl:if><xsl:if test="count(member) > 0">
import rpi.types.ComplexType;

/**
 * Composed type <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>
 * a.k.a <xsl:value-of select="@name" />
 */
public class <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template> extends ComplexType {
<xsl:apply-templates select="member" mode="decl" />
	
	/**
	 * Creates an empty <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>
	 */
	public <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>() {
	}
	<xsl:variable name="params"><xsl:for-each select="member"><xsl:call-template name="JavaType"><xsl:with-param name="name" select="@type"/></xsl:call-template><xsl:text> </xsl:text><xsl:value-of select="@name" /><xsl:text>, </xsl:text></xsl:for-each></xsl:variable>
	/**
	 * Creates an <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>
	 * <xsl:for-each select="member">
	 * @param <xsl:value-of select="@name" />
	 *            <xsl:value-of select="@description" />
</xsl:for-each>
	 */
	public <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>(<xsl:value-of select="substring($params, 0, string-length($params)-1)" />) {<xsl:apply-templates select="member" mode="assign" />
	}		

	public <xsl:call-template name="JavaName"><xsl:with-param name="name" select="@name" /></xsl:call-template>(String value) {
		this();
		consumeString(value);
	}
<xsl:apply-templates select="member" mode="getset" />

	@Override
	protected void appendComponents(StringBuilder buf) {<xsl:for-each select="member">
<xsl:if test="position() != 1">
		buf.append(",");</xsl:if>
		appendComponent(buf, "<xsl:value-of select="@name" />", <xsl:value-of select="@name" />);</xsl:for-each>
	}
	
	@Override
	protected String consumeComponent(String key, String value) {<xsl:for-each select="member">
		if(key.equals("<xsl:value-of select="@name"/>")) {
			return <xsl:value-of select="@name" />.consumeString(value);
		}</xsl:for-each>
		throw new IllegalArgumentException("key");
	}
}
</xsl:if>
</pre>
</body></html>
</xsl:template>

<xsl:template name="TypeName"><xsl:param name="name" /><xsl:choose>
	<xsl:when test="substring($name, string-length($name)-1)='[]'">RPI<xsl:value-of select="substring($name, 0, string-length($name)-1)" />Array</xsl:when>
	<xsl:otherwise>RPI<xsl:value-of select="$name" /></xsl:otherwise>
</xsl:choose></xsl:template>

<xsl:template name="JavaPackage"><xsl:param name="name" /><xsl:value-of select="substring-before($name,'::')" /></xsl:template>

<xsl:template name="JavaName"><xsl:param name="name" /><xsl:choose>
	<xsl:when test="substring($name, string-length($name)-1)='[]'">RPI<xsl:value-of select="substring-after(substring($name, 0, string-length($name)-1), '::')" />Array</xsl:when>
	<xsl:otherwise>RPI<xsl:value-of select="substring-after($name,'::')" /></xsl:otherwise>
</xsl:choose></xsl:template>

<xsl:template name="JavaType"><xsl:param name="name" />rpi.types.<xsl:call-template name="JavaPackage"><xsl:with-param name="name" select="$name" /></xsl:call-template>.<xsl:call-template name="JavaName"><xsl:with-param name="name" select="$name" /></xsl:call-template></xsl:template>

<xsl:template name="FirstUpper"><xsl:param name="text" /><xsl:value-of select="translate(substring($text,1,1),'abcdefghijklmnopqrstuvwxyz','ABCDEFGHIJKLMNOPQRSTUVWXYZ')" /><xsl:value-of select="substring($text,2)" /></xsl:template>

<xsl:template match="member" mode="decl">
	private <xsl:call-template name="JavaType"><xsl:with-param name="name" select="@type" /></xsl:call-template><xsl:text> </xsl:text><xsl:value-of select="@name" /> = new <xsl:call-template name="JavaType"><xsl:with-param name="name" select="@type" /></xsl:call-template>();</xsl:template>
	
<xsl:template match="member" mode="assign">
		this.<xsl:value-of select="@name" /> = <xsl:value-of select="@name" />;</xsl:template>
		
<xsl:template match="member" mode="getset">
	/**
	 * Sets the <xsl:value-of select="@description" />
	 * 
	 * @param value
	 *            The new value
	 */
	public void set<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>(<xsl:call-template name="JavaType"><xsl:with-param name="name" select="@type" /></xsl:call-template> value) {
		this.<xsl:value-of select="@name" /> = value;
	}
	
	/**
	 * Retrieves the <xsl:value-of select="@description" />
	 * 
	 * @return The current value
	 */
	public <xsl:call-template name="JavaType"><xsl:with-param name="name" select="@type" /></xsl:call-template> get<xsl:call-template name="FirstUpper"><xsl:with-param name="text" select="@name" /></xsl:call-template>() {
		return <xsl:value-of select="@name" />;
	}</xsl:template>
</xsl:stylesheet>