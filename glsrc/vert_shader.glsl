#version 110

attribute vec2 position;
uniform float rotation;
varying vec2 texcoord;
varying float zcoord;

void main()
{
    gl_Position = vec4(position[0]*cos(rotation), position[1], -sin(rotation)*position[0], 1.0);
    zcoord = gl_Position[2];
    texcoord = position * vec2(0.5) + vec2(0.5);
}


