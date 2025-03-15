#version 330 core

out vec4 FragColor;

in vec4 fragColor;
in vec3 fragPosition;
in vec3 fragNormal;
in vec2 fragUv;

flat in vec2 tiling;

uniform sampler2D albedo;
uniform int textureIndex = 0;
uniform int variant = 0;

vec3 ambientLight = vec3(0.20, 0.20, 0.22);

vec3 sunPosition = vec3(100.0, 200.0, 100.0);
vec3 sun_color = vec3(1.64, 1.27, 0.99);

vec3 indirect_color = vec3(0.8, 0.56, 0.40);

vec3 red_color = vec3(0.8, 0.0, 0.0);
vec3 blue_color = vec3(0.0, 0.0, 0.8);
vec3 huge_color = vec3(0.0, 0.8, 0.0);
vec3 rare_color = vec3(0.5, 0.5, 0.5);

void main() 
{
	vec3 lightDirection = normalize(sunPosition - fragPosition);
	float sunStrength = clamp(dot(fragNormal, lightDirection), 0.0, 1.0);
	vec3 sunlight = sunStrength * sun_color;

	vec3 indirect_light_dir = normalize(vec3(-lightDirection.x, 0.0, -lightDirection.z));
	float indirect_coefficient = clamp(dot(fragNormal, indirect_light_dir), 0.0, 1.0);
	vec3 indirect = indirect_coefficient * indirect_color;

	vec4 lighting = vec4(sunlight + indirect + ambientLight, 1.0);

	vec2 tiledUv = vec2(fract(fragUv.x * tiling.x), fract(fragUv.y * tiling.y));
	vec2 texUv = vec2(tiledUv.x * 0.25 + 0.25 * textureIndex, tiledUv.y);

	vec4 diffuse = texture(albedo, texUv);
	if (variant == 1) {
		diffuse += vec4(red_color, 0.0);
	}
	else if (variant == 2) {
		diffuse += vec4(blue_color, 0.0);
	}
	else if (variant == 3) {
		diffuse += vec4(huge_color, 0.0);
	}
	else if (variant == 3) {
		diffuse += vec4(rare_color, 0.0);
	}

	FragColor = lighting * diffuse;
}
