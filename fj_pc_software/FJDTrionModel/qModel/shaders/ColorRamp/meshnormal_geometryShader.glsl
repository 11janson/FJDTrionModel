#version 330 core


layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;



in vec3 outColor[];
in vec3 outPos[];
in uint isHidden[];

out vec3 geomColor;

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


bool calcutateInside(vec3 point)
{
	vec4 newpoint = boxMatrix* vec4(point, 1.0);
	if(newpoint[0] >= boxmincorner[0] && newpoint[0] <= boxmaxcorner[0] && newpoint[1] >= boxmincorner[1] && newpoint[1] <= boxmaxcorner[1] && newpoint[2] >= boxmincorner[2] && newpoint[2] <= boxmaxcorner[2])
	{
		return true;
	}
	return false;
}

bool isinsidebox(vec3 point1,vec3 point2,vec3 point3)
{
	if(calcutateInside(point1) && calcutateInside(point2) && calcutateInside(point3))
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
		if(!isinsidebox(outPos[0],outPos[1],outPos[2]))
		{
			isinside = false;
		}
	}
	if(isHidden[0] == uint(0) && isHidden[1] == uint(0) && isHidden[2] == uint(0) && isinside)
	{
		vec3 edge1 = vec3(outPos[0] - outPos[1]);
		vec3 edge2 = vec3(outPos[2] - outPos[1]);
		vec3 geomNormal = normalize(cross(edge1, edge2));
		for (int i = 0; i < 3; i++)
		{
			geomColor = outColor[i];

			if(showNormal)
			{
				vec4 lightDirection = vec4(normalize(lightPosition - outPos[i]), 0.0);
				vec4 normalVector = vec4(geomNormal, 0.0);
				vec4 cameraPosition = modelview * vec4(outPos[i], 1.0);
				vec3 ambient = ambientColor * outColor[i];
		
				float diffuse = max(dot(normalVector, lightDirection), 0.0);
				vec3 diffuseVal = diffuseCoefficient * diffuseColor * diffuse* outColor[i];

				vec3 viewDirection = normalize(-cameraPosition.xyz);
				vec3 halfVector = normalize(lightDirection.xyz + viewDirection);
				float specular = pow(max(dot(halfVector, geomNormal), 0.0), 32.0);
				vec3 specularVal = 0.8 * vec3(1.0,1.0,1.0) * specular * outColor[i];
				geomColor = ambient + diffuseVal + specularVal;	
			}		
			gl_Position = gl_in[i].gl_Position;
			EmitVertex();
		}	
		EndPrimitive();
	}

};