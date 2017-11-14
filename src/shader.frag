#version 330 core

precision mediump float;

out vec4 out_fragcolor;

in vec3 vcolor;
in vec3 vnormal;

uniform vec3 viewdir;
uniform vec3 light;

void main() {
	vec3 L = -normalize(light);

	vec3 N;
	if (gl_FrontFacing)
		N = vnormal;
	else
		N = -vnormal;

	vec3 V = -normalize(viewdir);
	vec3 H = normalize(L + V);
	float ambient = .2;
	float diffuse = .7 * max(0.0, dot(N, L));
	float spec = .1 * pow(max(0.0, dot(N, H)), 12);
	vec4 phong = vec4(vcolor * ambient + vcolor * diffuse + vec3(1.0) * spec, 1.0);

	out_fragcolor = phong;
	//out_fragcolor = clamp(phong, 0.0, 1.0) + vec4(N, 0);
}
