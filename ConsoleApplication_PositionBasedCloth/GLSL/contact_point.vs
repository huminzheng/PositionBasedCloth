# version 330

layout (location = 0) in vec3 position;
layout (location = 1) in uint type;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out float typef;

void main()
{
	gl_Position = projection * view * model * vec4(position, 1.0f);
	typef = (float(type) - 20.0f) / 2.0f;
}