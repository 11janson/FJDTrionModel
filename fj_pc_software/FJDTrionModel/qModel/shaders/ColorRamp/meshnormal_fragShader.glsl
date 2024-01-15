#version 330 core

in vec3 geomColor;
out vec4 FragColor;

void main()
{
    FragColor = vec4(geomColor,1.0f);
	if(!gl_FrontFacing)
	{
        FragColor = vec4(0.31f, 0.31f, 0.31f, 1.0f);
    } 
};