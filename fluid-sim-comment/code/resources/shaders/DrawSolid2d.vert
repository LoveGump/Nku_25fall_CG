#version 330

layout(location = 0) in vec2 position;

uniform float scale;

void main()
{
    gl_Position = vec4(position.xy / scale, 0.0, 1.0);
}
