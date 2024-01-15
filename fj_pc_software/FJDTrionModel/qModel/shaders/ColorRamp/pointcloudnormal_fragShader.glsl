#version 330 core


in vec4 outColor;
in vec3 outPos;

out vec4 FragColor;

uniform vec3 lightPosition;
uniform vec3 ambientColor;  
uniform vec3 diffuseColor;  
uniform float diffuseCoefficient;  
uniform mat4 modelview;
uniform bool showNormal;
uniform bool useclipbox;
uniform vec3 boxmincorner;
uniform vec3 boxmaxcorner;
uniform mat4 boxMatrix;
uniform int decimStep;
uniform float colorBright;
uniform float colorContrast;

float updateColor(float fColor)
{
	float fBright = colorBright;
	float fContrast = colorContrast;
	float threshold = 1.0;
	

	//[!].亮度
	float sColor = clamp(fColor + fBright, 0.0, 1.0);

	//[!].对比度
	if (fContrast >= 1.0)
	{
		 sColor = sColor >= threshold ? threshold : 0;
	}
	else 
	{
		float cv = 1.0f / (1.0f - fContrast) - 1.0f;
		sColor = sColor + (sColor - threshold) * cv;
	}

	return clamp(sColor, 0.0, 1.0);

}

//[!].更新对比度亮度调节
vec4 updateContrastBrightness(vec4 sColor)
{
	
	vec4 colorValue;
	colorValue.r = updateColor(sColor.r);
	colorValue.g = updateColor(sColor.g);
	colorValue.b = updateColor(sColor.b);
	colorValue.a = sColor.a;
	return colorValue;
}

bool calcutateInside(vec3 point)
{
	vec4 newpoint = boxMatrix* vec4(point, 1.0);
	if(newpoint[0] >= boxmincorner[0] && newpoint[0] <= boxmaxcorner[0] && newpoint[1] >= boxmincorner[1] && newpoint[1] <= boxmaxcorner[1] && newpoint[2] >= boxmincorner[2] && newpoint[2] <= boxmaxcorner[2])
	{
		return true;
	}
	return false;
}

bool isinsidebox(vec3 point1)
{
	if(calcutateInside(point1))
	{
		return true;
	}
	return false;
}

void main()
{
	bool isinside = true;
	if(useclipbox)
	{
		if(!isinsidebox(outPos))
		{
			isinside = false;
		}
	}
	if(decimStep > 1)
	{
		if((uint(outColor[3]) - uint(1)) % uint(decimStep) != uint(0) )
		{
			isinside = false;
		}
	}

	if(isinside && outColor[3] >0 )
	{
		vec3 geomColor = vec3(outColor[0],outColor[1],outColor[2]);	
		if(showNormal)
		{
			vec4 lightDirection = vec4(normalize(lightPosition - outPos), 0.0);
			vec4 normalVector = vec4(outPos, 0.0);
			vec4 cameraPosition = modelview * vec4(outPos, 1.0);
			vec3 ambient = ambientColor * geomColor;		
			float diffuse = max(dot(normalVector, lightDirection), 0.0);
			vec3 diffuseVal = diffuseCoefficient * diffuseColor * diffuse* geomColor;

			vec3 viewDirection = normalize(-cameraPosition.xyz);
			vec3 halfVector = normalize(lightDirection.xyz + viewDirection);
			float specular = pow(max(dot(halfVector, outPos), 0.0), 32.0);
			vec3 specularVal = 0.8 * vec3(1.0,1.0,1.0) * specular * geomColor;
			geomColor = ambient + diffuseVal + specularVal;	
		}	
		FragColor = updateContrastBrightness(vec4(geomColor, outColor[3]));
	}
	else
	{
		discard; 
	}
};