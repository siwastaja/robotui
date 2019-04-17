#version 130
in vec3 vertex_position;
in vec4 vertex_color;

out vec4 frag_color;

uniform mat4 pvm;

void main()
{
	gl_Position = pvm * vec4(vertex_position, 1.0);
	frag_color = vertex_color;
}
