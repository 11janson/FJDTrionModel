#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor;

out vec4 outColor;
out vec3 outPos;

uniform mat4 view;
uniform mat4 projection;
uniform float adaptivePointSize;

void main()
{
	outPos = aPos;
    gl_Position = projection * view * vec4(aPos, 1.0);
    outColor = aColor;
    gl_PointSize = adaptivePointSize;
}