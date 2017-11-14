#version 330 core

precision mediump float;

in vec3 in_position;
in vec3 in_color;
in vec3 in_normal;

out vec3 vnormal;
out vec3 vcolor;

uniform int pcolor;
uniform vec3 viewdir;
uniform vec3 light;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	vec4 pos = projection * view * vec4(in_position,1.0);
	
	if (pcolor > .5) {
		vcolor = vec3(1, 1, 1);
	}
	else {
		vcolor = in_color;
	}
	vnormal = in_normal;
	gl_Position = pos;
}
