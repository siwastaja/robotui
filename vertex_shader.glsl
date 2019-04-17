#version 130
in vec4 vertex_position;
in vec4 vertex_color;

out vec4 frag_color;

uniform mat4 pvm;

void main()
{
	gl_Position = pvm * vec4(vertex_position.x, vertex_position.y, vertex_position.z, 1.0);
	frag_color = vertex_color;
}
