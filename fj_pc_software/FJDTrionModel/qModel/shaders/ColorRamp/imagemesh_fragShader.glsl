#version 330 core

in vec2 tc;
out vec4 fragColor;

uniform sampler2D textureSampler;
uniform float transparency;
void main(void)
{	
	fragColor = texture(textureSampler,tc);
	if(fragColor[3] == 0)
	{
		discard;
	}
	else if(transparency < 1.0)
	{
		fragColor[3] = transparency;
	}
	//fragColor = vec4(1,1,1,1);
}
