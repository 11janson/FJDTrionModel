#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec2 tex_coord;
out vec2 tc;

uniform mat4 view;
uniform mat4 projection;
void main(void)
{	
    gl_Position = projection * view * vec4(position, 1.0);
	tc = tex_coord;
}