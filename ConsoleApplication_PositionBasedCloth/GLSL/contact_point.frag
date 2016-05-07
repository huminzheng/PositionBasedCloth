# version 330 core

in float typef;
out vec4 color;

void main() 
{
	color = vec4(1.0f - typef, 1.0f, typef, 1.0f);
}