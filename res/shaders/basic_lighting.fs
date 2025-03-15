#version 330 core

out vec4 FragColor;

in vec4 fragColor;
in vec3 fragPosition;
in vec3 fragNormal;
in vec2 fragUv;

flat in vec2 tiling;

uniform sampler2D albedo;
uniform int textureIndex = 0;

vec3 sunPosition = vec3(100.0, 200.0, 100.0);
vec3 sun_color = vec3(1.64, 1.27, 0.99);

vec3 indirect_color = vec3(0.4, 0.28, 0.20);

void main() 
{
	vec3 lightDirection = normalize(sunPosition - fragPosition);
	float sunStrength = clamp(dot(fragNormal, lightDirection), 0.0, 1.0);
	vec3 sunlight = sunStrength * sun_color;

	vec3 indirect_light_dir = normalize(vec3(-lightDirection.x, 0.0, -lightDirection.z));
	float indirect_coefficient = clamp(dot(fragNormal, indirect_light_dir), 0.0, 1.0);
	vec3 indirect = indirect_coefficient * indirect_color;

	vec4 lighting = vec4(sunlight + indirect, 1.0);

	vec2 tiledUv = vec2(fract(fragUv.x * tiling.x), fract(fragUv.y * tiling.y));
	vec2 texUv = vec2(tiledUv.x * 0.25 + 0.25 * textureIndex, tiledUv.y);

	vec4 diffuse = texture(albedo, texUv);
	FragColor = lighting * diffuse;
}
