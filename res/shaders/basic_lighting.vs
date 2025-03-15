#version 330 core

out vec4 fragColor;
out vec3 fragPosition;
out vec3 fragNormal;
out vec2 fragUv;

flat out vec2 tiling;

in vec3 position;
in vec4 vertexColor;
in vec3 vertexNormal; // There is an implicit mapping defined for the locations, but they also match input by name, cool!
in vec2 vertexUv;

uniform mat4 mvp;
uniform mat4 matModel;
uniform vec3 meshDimensions = vec3(1.0, 1.0, 1.0);

vec2 get_tiling_factor(vec3 normal, vec3 dimensions)
{
	if (abs(normal.x) > 0.001) {
		return vec2(dimensions.y, dimensions.z);
	}

	if (abs(normal.y) > 0.001) {
		return vec2(dimensions.x, dimensions.z);
	}

	if (abs(normal.z) > 0.001) {
		return vec2(dimensions.y, dimensions.x);
	}

	return vec2(1.0, 1.0);
}

void main() 
{
	fragPosition = vec3(matModel * vec4(position, 1.0));
	fragNormal = vertexNormal;
	fragColor = vertexColor;
	fragUv = vertexUv;

	tiling = get_tiling_factor(vertexNormal, meshDimensions);

	gl_Position = mvp * vec4(position, 1.0);
}
