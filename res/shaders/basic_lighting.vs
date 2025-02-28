#version 330 core

out vec4 fragColor;
out vec3 fragPosition;
out vec3 fragNormal;
out vec2 fragUv;

in vec3 position;
in vec4 vertexColor;
in vec3 vertexNormal; // There is an implicit mapping defined for the locations, but they also match input by name, cool!
in vec2 vertexUv;

uniform mat4 mvp;
uniform mat4 matModel;

void main() 
{
	fragPosition = vec3(matModel * vec4(position, 1.0));
	fragNormal = vertexNormal;
	fragColor = vertexColor;
	fragUv = vertexUv;

	gl_Position = mvp * vec4(position, 1.0);
}
